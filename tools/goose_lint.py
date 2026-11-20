#!/usr/bin/env python3
"""
goose-lint: style checker / fixer for the GOOSE C and C++ embedded code base.

Default mode reports style violations (gcc-style output, so vim's quickfix and
most editors can jump to them) and exits non-zero if any errors are found.
With --fix it repairs everything that can be repaired mechanically, then
reports whatever is left for the programmer to fix by hand.

Usage:
    python3 tools/goose_lint.py                 # lint the whole repo (report only)
    python3 tools/goose_lint.py src/ include/   # lint specific paths
    python3 tools/goose_lint.py --fix           # auto-fix, then report the rest
    python3 tools/goose_lint.py --staged        # only files staged in git
    python3 tools/goose_lint.py --list-rules    # show every rule
    python3 tools/goose_lint.py --install-hook  # run on every git commit

Suppressing a finding (put it in a comment):
    foo();  // goose-lint: disable=blocking-delay
    // goose-lint: disable-next-line=dynamic-alloc,goto
    // goose-lint: disable-file=fixed-width-int      (anywhere in the file)
    Use "all" to suppress every rule.

Needs only Python 3.8+. clang-format is used if installed (optional).
"""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys

# --------------------------------------------------------------------------
# Defaults (override in .gooselint.json at the repo root)
# --------------------------------------------------------------------------

DEFAULT_CONFIG = {
    "extensions": [".c", ".h", ".cpp", ".hpp", ".cc", ".hh", ".cxx", ".hxx", ".ino"],
    "header_extensions": [".h", ".hpp", ".hh", ".hxx"],
    "exclude": ["build/", ".pio/", "lib/", "third_party/", "external/", ".git/"],
    "max_line_length": 80,
    "max_function_lines": 60,
    "indent_width": 4,
    "max_blank_lines": 2,
    "use_clang_format": True,
    "clang_format_binary": "clang-format",
    "naming": {
        "macro": "^[A-Z][A-Z0-9_]*$",
        "type": "^[A-Z][A-Za-z0-9]*$",
    },
    "disabled_rules": [],
    "severity_overrides": {},
}

# id: (default severity, auto-fixable, description)
RULES = {
    "crlf":                   ("error",   True,  "Use LF line endings, not CRLF"),
    "tabs":                   ("error",   True,  "Indent with spaces, not tabs"),
    "trailing-whitespace":    ("error",   True,  "No whitespace at end of line"),
    "eof-newline":            ("error",   True,  "File must end with exactly one newline"),
    "blank-lines":            ("error",   True,  "Too many consecutive blank lines"),
    "keyword-space":          ("error",   True,  "Put a space between if/for/while/switch and '('"),
    "header-guard":           ("error",   True,  "Headers need '#pragma once' or an include guard"),
    "clang-format":           ("error",   True,  "Formatting differs from .clang-format"),
    "brace-style":            ("error",   True,  "Opening brace goes on the same line as its statement"),
    "line-length":            ("error",   False, "Line longer than max_line_length"),
    "function-length":        ("error",   False, "Function body longer than max_function_lines"),
    "braces":                 ("error",   False, "if/else/for/while bodies must use braces"),
    "macro-naming":           ("error",   False, "Macros must be UPPER_SNAKE_CASE"),
    "type-naming":            ("error",   False, "class/struct/enum/union names must be PascalCase"),
    "dynamic-alloc":          ("error",   False, "No heap allocation (malloc/free/new/delete) in flight code"),
    "unsafe-string":          ("error",   False, "No unbounded string functions (sprintf/strcpy/strcat/gets)"),
    "goto":                   ("error",   False, "No goto"),
    "using-namespace-header": ("error",   False, "No 'using namespace' in headers"),
    "blocking-delay":         ("warning", False, "delay()/delayMicroseconds() block the control loop"),
    "fixed-width-int":        ("warning", False, "Prefer <cstdint> types (uint32_t etc.) over short/long/unsigned"),
    "todo-format":            ("warning", False, "Write TODOs as TODO(name): ..."),
}

# --------------------------------------------------------------------------
# Lexing: classify each character as code, comment or string literal
# --------------------------------------------------------------------------

CODE, COMMENT, STRING = "c", "m", "s"


def classify(text):
    """Return a list (same length as text) of CODE/COMMENT/STRING per char."""
    n = len(text)
    cls = [CODE] * n
    i = 0
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if ch == "/" and nxt == "/":
            j = i
            while j < n and text[j] != "\n":
                # line continuation keeps a // comment going
                if text[j] == "\\" and j + 1 < n and text[j + 1] == "\n":
                    cls[j] = COMMENT
                    j += 2
                    continue
                cls[j] = COMMENT
                j += 1
            i = j
        elif ch == "/" and nxt == "*":
            end = text.find("*/", i + 2)
            end = n if end == -1 else end + 2
            for k in range(i, end):
                if text[k] != "\n":
                    cls[k] = COMMENT
            i = end
        elif ch == "R" and nxt == '"' and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            # raw string R"delim( ... )delim"
            paren = text.find("(", i + 2)
            if paren == -1:
                i += 1
                continue
            delim = text[i + 2:paren]
            end = text.find(")" + delim + '"', paren)
            end = n if end == -1 else end + len(delim) + 2
            for k in range(i, end):
                if text[k] != "\n":
                    cls[k] = STRING
            i = end
        elif ch == '"' or (ch == "'" and not (i > 0 and text[i - 1].isalnum())):
            # string or char literal (the isalnum check skips C++14 digit separators 1'000)
            q = ch
            j = i + 1
            while j < n and text[j] != q and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            end = min(j + 1, n)
            for k in range(i, end):
                if text[k] != "\n":
                    cls[k] = STRING
            i = end
        else:
            i += 1
    return cls


def masked(text, cls, keep):
    """Text with every char not of class `keep` replaced by a space (newlines kept)."""
    return "".join(c if (k == keep or c == "\n") else " " for c, k in zip(text, cls))


def line_starts(text):
    starts = [0]
    for i, c in enumerate(text):
        if c == "\n":
            starts.append(i + 1)
    return starts


def offset_to_linecol(starts, off):
    lo, hi = 0, len(starts) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if starts[mid] <= off:
            lo = mid
        else:
            hi = mid - 1
    return lo + 1, off - starts[lo] + 1


# --------------------------------------------------------------------------
# Violations and suppressions
# --------------------------------------------------------------------------

class Violation(object):
    __slots__ = ("path", "line", "col", "rule", "severity", "message", "fixable")

    def __init__(self, path, line, col, rule, message, cfg):
        self.path = path
        self.line = line
        self.col = col
        self.rule = rule
        self.severity = cfg["severity_overrides"].get(rule, RULES[rule][0])
        self.message = message
        self.fixable = RULES[rule][1]

    def __str__(self):
        tag = " (fixable with --fix)" if self.fixable else ""
        return "%s:%d:%d: %s: %s [%s]%s" % (
            self.path, self.line, self.col, self.severity, self.message, self.rule, tag)


DIRECTIVE_RE = re.compile(r"goose-lint:\s*(disable-next-line|disable-file|disable)\s*=\s*([\w\-, ]+)")


def parse_suppressions(comment_text):
    per_line = {}       # line number -> set of rules
    whole_file = set()
    for lineno, line in enumerate(comment_text.split("\n"), start=1):
        for m in DIRECTIVE_RE.finditer(line):
            kind = m.group(1)
            rules = {r.strip() for r in m.group(2).split(",") if r.strip()}
            if kind == "disable-file":
                whole_file |= rules
            else:
                target = lineno + 1 if kind == "disable-next-line" else lineno
                per_line.setdefault(target, set()).update(rules)
    return per_line, whole_file


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------

KEYWORD_PAREN_RE = re.compile(r"\b(if|for|while|switch)\(")
CONTROL_RE = re.compile(r"\b(if|for|while)\s*(?:constexpr\s*)?\(")
ELSE_RE = re.compile(r"\belse\b")
MACRO_RE = re.compile(r"^[ \t]*#[ \t]*define[ \t]+(\w+)", re.M)
TYPE_RE = re.compile(r"\b(class|struct|union|enum(?:\s+class|\s+struct)?)\s+(\w+)\s*(?:final\s*)?(?=[{;\n]|:(?!:))")
# placement new  "new (buffer) T"  is allowed, so 'new' followed by '(' is skipped
ALLOC_RE = re.compile(r"\b(malloc|calloc|realloc|free)\s*\(|\bnew\b(?!\s*\()|\bdelete\b")
DELETE_OK_RE = re.compile(r"=\s*delete\b")
UNSAFE_RE = re.compile(r"\b(sprintf|vsprintf|strcpy|strcat|gets)\s*\(")
GOTO_RE = re.compile(r"\bgoto\b")
DELAY_RE = re.compile(r"\b(delay|delayMicroseconds)\s*\(")
INT_RE = re.compile(r"\b(?:unsigned|signed)\s+(?:long\s+long|long|int|short|char)\b"
                    r"|\b(?:long\s+long|unsigned|short|long)\b")
USING_NS_RE = re.compile(r"\busing\s+namespace\b")
TODO_RE = re.compile(r"\b(TODO|FIXME|XXX|HACK)\b(?!\([\w.\- ]+\):)")
PRAGMA_ONCE_RE = re.compile(r"^[ \t]*#[ \t]*pragma[ \t]+once\b", re.M)
GUARD_RE = re.compile(r"^[ \t]*#[ \t]*ifndef[ \t]+(\w+)[ \t]*\n[ \t]*#[ \t]*define[ \t]+\1\b", re.M)


def is_preproc_line(code_lines, lineno):
    return code_lines[lineno - 1].lstrip().startswith("#")


def lint_text(path, text, cfg, fix_clang_ok=False):
    """Return a list of Violations for one file's contents."""
    out = []
    ext = os.path.splitext(path)[1].lower()
    is_header = ext in cfg["header_extensions"]

    def add(line, col, rule, msg):
        out.append(Violation(path, line, col, rule, msg, cfg))

    # ---- raw text checks -------------------------------------------------
    if "\r\n" in text:
        add(text[:text.index("\r\n")].count("\n") + 1, 1, "crlf", RULES["crlf"][2])
    text = text.replace("\r\n", "\n")

    cls = classify(text)
    code = masked(text, cls, CODE)
    comments = masked(text, cls, COMMENT)
    starts = line_starts(text)
    raw_lines = text.split("\n")
    code_lines = code.split("\n")

    blank_run = 0
    for lineno, line in enumerate(raw_lines, start=1):
        if "\t" in line:
            add(lineno, line.index("\t") + 1, "tabs", RULES["tabs"][2])
        stripped = line.rstrip(" \t")
        if stripped != line:
            add(lineno, len(stripped) + 1, "trailing-whitespace", RULES["trailing-whitespace"][2])
        width = len(line.expandtabs(cfg["indent_width"]))
        if width > cfg["max_line_length"]:
            add(lineno, cfg["max_line_length"] + 1, "line-length",
                "Line is %d characters (max %d)" % (width, cfg["max_line_length"]))
        if stripped == "" and lineno < len(raw_lines):
            blank_run += 1
            if blank_run == cfg["max_blank_lines"] + 1:
                add(lineno, 1, "blank-lines",
                    "More than %d consecutive blank lines" % cfg["max_blank_lines"])
        else:
            blank_run = 0

    if text and (not text.endswith("\n") or text.endswith("\n\n")):
        add(len(raw_lines), 1, "eof-newline", RULES["eof-newline"][2])

    # ---- code checks (comments and strings masked out) ------------------
    for m in KEYWORD_PAREN_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        if not is_preproc_line(code_lines, ln):
            add(ln, col, "keyword-space", "Missing space in '%s('" % m.group(1))

    for ln, col, kw in find_unbraced(code, starts, code_lines):
        add(ln, col, "braces", "'%s' body must be wrapped in { }" % kw)

    for ln in find_detached_braces(code_lines):
        line_code = code_lines[ln - 1]
        if line_code.strip().startswith("else"):
            add(ln, line_code.index("else") + 1, "brace-style", "Write '} else {' on one line")
        else:
            add(ln, line_code.index("{") + 1, "brace-style",
                "Put '{' at the end of the previous line, not on its own line")

    for name, ln, length in find_functions(code, starts):
        if length > cfg["max_function_lines"]:
            add(ln, 1, "function-length",
                "Function '%s' is %d lines (max %d); split it into helpers"
                % (name, length, cfg["max_function_lines"]))

    macro_re = re.compile(cfg["naming"]["macro"])
    for m in MACRO_RE.finditer(code):
        name = m.group(1)
        if not macro_re.match(name):
            ln, col = offset_to_linecol(starts, m.start(1))
            add(ln, col, "macro-naming", "Macro '%s' should be UPPER_SNAKE_CASE" % name)

    type_re = re.compile(cfg["naming"]["type"])
    for m in TYPE_RE.finditer(code):
        name = m.group(2)
        ln, col = offset_to_linecol(starts, m.start(2))
        if is_preproc_line(code_lines, ln):
            continue
        if not type_re.match(name):
            kind = m.group(1).split()[0]
            add(ln, col, "type-naming", "%s '%s' should be PascalCase" % (kind, name))

    for m in ALLOC_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        word = m.group(1) or m.group(0).strip()
        if word == "delete":
            # '= delete;' on special member functions is fine
            line_code = code_lines[ln - 1]
            if DELETE_OK_RE.search(line_code[:col + 6]):
                continue
        add(ln, col, "dynamic-alloc",
            "'%s' uses the heap; use static/stack storage or a fixed pool" % word)

    for m in UNSAFE_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        alt = {"sprintf": "snprintf", "vsprintf": "vsnprintf", "strcpy": "strncpy/strlcpy",
               "strcat": "strncat/strlcat", "gets": "fgets"}[m.group(1)]
        add(ln, col, "unsafe-string", "'%s' can overflow; use %s" % (m.group(1), alt))

    for m in GOTO_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        add(ln, col, "goto", RULES["goto"][2])

    for m in DELAY_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        add(ln, col, "blocking-delay",
            "'%s()' blocks; schedule with elapsedMicros/millis() timing instead" % m.group(1))

    for m in INT_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        if is_preproc_line(code_lines, ln):
            continue
        add(ln, col, "fixed-width-int",
            "'%s' has platform-dependent width; use a <cstdint> type" % " ".join(m.group(0).split()))

    if is_header:
        for m in USING_NS_RE.finditer(code):
            ln, col = offset_to_linecol(starts, m.start())
            add(ln, col, "using-namespace-header", RULES["using-namespace-header"][2])
        if not PRAGMA_ONCE_RE.search(code) and not GUARD_RE.search(code):
            add(1, 1, "header-guard", RULES["header-guard"][2])

    for m in TODO_RE.finditer(comments):
        ln, col = offset_to_linecol(starts, m.start())
        add(ln, col, "todo-format", "Write '%s(name): ...' so it has an owner" % m.group(1))

    # ---- filter disabled / suppressed ------------------------------------
    per_line, whole_file = parse_suppressions(comments)
    disabled = set(cfg["disabled_rules"])

    def keep(v):
        if v.rule in disabled or v.rule in whole_file or "all" in whole_file:
            return False
        s = per_line.get(v.line, ())
        return v.rule not in s and "all" not in s

    return [v for v in out if keep(v)]


BRACE_PREV_OK = (";", "{", "}", ":")   # a '{' after these opens a plain scope


def find_detached_braces(code_lines):
    """Yield line numbers where '{' (or 'else') starts a line but belongs on the line above."""
    prev = None  # index of previous non-blank, non-preprocessor code line
    for i, line in enumerate(code_lines):
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            prev = None
            continue
        if prev is not None:
            p = code_lines[prev].strip()
            if s.startswith("{"):
                # a bare '{' after ';' / '{' / ':' / a lone '}' is a plain scope block
                if not p.endswith(BRACE_PREV_OK) or (p.endswith("}") and p != "}"):
                    yield i + 1
            elif re.match(r"else\b", s) and p == "}":
                yield i + 1   # want '} else {'
        prev = i


QUALIFIERS = {"const", "override", "noexcept", "final", "mutable", "volatile"}
SCOPE_RE = re.compile(r"\b(namespace|class|struct|union|enum|extern)\b")
CONTROL_HEAD_RE = re.compile(r"^\s*(if|else|for|while|switch|do|catch|try)\b")
FUNC_NAME_RE = re.compile(r"([~\w:]+)\s*\(")


def find_functions(code, starts):
    """Yield (name, start_line, length_in_lines) for each top-level function body.

    Heuristic brace scanner: a '{' is a function body when the text before it
    (back to the previous ';', '{' or '}') has a parameter list, isn't a
    control statement, and isn't a brace initializer. Length counts lines from
    the line of '{' to the line of the matching '}', inclusive.
    """
    stack = []          # entries: "func", "scope", "block", "init"
    seg = 0             # start of the current statement head
    open_line = {}      # depth -> (name, line) for open functions
    last_init_close = -1  # offset of the last '}' that closed a brace initializer
    for i, ch in enumerate(code):
        if ch == ";":
            if not stack or stack[-1] != "init":
                seg = i + 1
        elif ch == "{":
            head = code[seg:i]
            head_lines = [l for l in head.split("\n") if not l.strip().startswith("#")]
            head = " ".join(head_lines)
            tail = head.rstrip()
            last_tok = re.search(r"(\w+)$", tail)
            in_func = "func" in stack
            # ctor init list ending in a brace initializer:  Foo() : a_(1), b_{2} {
            after_init = last_init_close == seg + len(code[seg:i].rstrip()) - 1
            if SCOPE_RE.search(head) and "(" not in head:
                kind = "scope"
            elif ("(" in head and not CONTROL_HEAD_RE.match(head) and "=" not in head.split("(")[0]
                  and (tail.endswith(")") or "->" in head or after_init
                       or (last_tok and last_tok.group(1) in QUALIFIERS))):
                kind = "block" if in_func else "func"
            elif tail.endswith(("=", ",", "(", "[")) or (last_tok and not CONTROL_HEAD_RE.match(head)
                                                         and last_tok.group(1) not in ("do", "else", "try")):
                kind = "init"
            else:
                kind = "block"
            if kind == "func":
                m = None
                for m in FUNC_NAME_RE.finditer(head.split(")")[0] + ")"):
                    pass
                name = m.group(1) if m else "?"
                open_line[len(stack)] = (name, offset_to_linecol(starts, i)[0])
            stack.append(kind)
            if kind != "init":
                seg = i + 1
        elif ch == "}":
            if not stack:
                continue
            kind = stack.pop()
            if kind == "func":
                name, start = open_line.pop(len(stack))
                end = offset_to_linecol(starts, i)[0]
                yield name, start, end - start + 1
            if kind == "init":
                last_init_close = i
            else:
                seg = i + 1


def find_unbraced(code, starts, code_lines):
    """Yield (line, col, keyword) for if/for/while/else bodies without braces."""
    n = len(code)
    for m in CONTROL_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        if is_preproc_line(code_lines, ln):
            continue
        # walk to the matching close paren
        depth, i = 0, m.end() - 1
        while i < n:
            if code[i] == "(":
                depth += 1
            elif code[i] == ")":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        j = i + 1
        while j < n and code[j] in " \t\n":
            j += 1
        if j >= n:
            continue
        nxt = code[j]
        if nxt == "{":
            continue
        if nxt == ";" and m.group(1) == "while":
            continue  # do { } while (x);  or an intentional empty spin-wait
        yield ln, col, m.group(1)

    for m in ELSE_RE.finditer(code):
        ln, col = offset_to_linecol(starts, m.start())
        j = m.end()
        while j < n and code[j] in " \t\n":
            j += 1
        if j < n and code[j] != "{" and not code.startswith("if", j):
            yield ln, col, "else"


# --------------------------------------------------------------------------
# Fixers (each is text -> text; run in order)
# --------------------------------------------------------------------------

def fix_crlf(text, cfg, path):
    return text.replace("\r\n", "\n")


def fix_tabs(text, cfg, path):
    cls = classify(text)
    out, col = [], 0
    w = cfg["indent_width"]
    for c, k in zip(text, cls):
        if c == "\t" and k != STRING:
            spaces = w - (col % w)
            out.append(" " * spaces)
            col += spaces
        else:
            out.append(c)
            col = 0 if c == "\n" else col + 1
    return "".join(out)


def fix_trailing(text, cfg, path):
    cls = classify(text)
    lines, pos, out = text.split("\n"), 0, []
    for line in lines:
        stripped = line.rstrip(" \t")
        # don't touch whitespace that is inside a (raw) string literal
        if stripped != line and cls[pos + len(stripped)] == STRING:
            out.append(line)
        else:
            out.append(stripped)
        pos += len(line) + 1
    return "\n".join(out)


def fix_keyword_space(text, cfg, path):
    cls = classify(text)
    code = masked(text, cls, CODE)
    starts = line_starts(text)
    code_lines = code.split("\n")
    inserts = []
    for m in KEYWORD_PAREN_RE.finditer(code):
        ln, _ = offset_to_linecol(starts, m.start())
        if not is_preproc_line(code_lines, ln):
            inserts.append(m.end() - 1)
    for off in reversed(inserts):
        text = text[:off] + " " + text[off:]
    return text


def fix_brace_style(text, cfg, path):
    code = masked(text, classify(text), CODE)
    lines, code_lines = text.split("\n"), code.split("\n")
    for ln in reversed(list(find_detached_braces(code_lines))):
        i = ln - 1
        p = i - 1
        while code_lines[p].strip() == "":
            p -= 1
        if code_lines[p].strip().startswith("#"):
            continue
        # join onto the code part of the previous line (before any // comment)
        cut = len(code_lines[p].rstrip())
        comment = lines[p][cut:].strip()
        if code_lines[i].strip().startswith("else"):
            moved, rest = lines[i].strip(), ""          # whole 'else ... {' line moves up
        else:
            moved, rest = "{", lines[i].lstrip()[1:].strip()
        lines[p] = lines[p][:cut] + " " + moved + (" " + comment if comment else "")
        code_lines[p] = code_lines[p][:cut] + " " + moved
        if rest:
            indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
            lines[i] = indent + " " * cfg["indent_width"] + rest
        else:
            del lines[i]
            del code_lines[i]
    return "\n".join(lines)


def fix_blank_lines(text, cfg, path):
    limit = cfg["max_blank_lines"]
    out, run = [], 0
    for line in text.split("\n"):
        if line.strip() == "":
            run += 1
            if run > limit:
                continue
        else:
            run = 0
        out.append(line)
    return "\n".join(out)


def fix_eof(text, cfg, path):
    return text.rstrip("\n") + "\n" if text.strip() else text


def fix_header_guard(text, cfg, path):
    if os.path.splitext(path)[1].lower() not in cfg["header_extensions"]:
        return text
    code = masked(text, classify(text), CODE)
    if PRAGMA_ONCE_RE.search(code) or GUARD_RE.search(code):
        return text
    # insert after any leading comment block / blank lines
    lines = text.split("\n")
    code_lines = code.split("\n")
    i = 0
    while i < len(lines) and code_lines[i].strip() == "":
        i += 1
    lines.insert(i, "#pragma once")
    if i + 1 < len(lines) and lines[i + 1].strip() != "":
        lines.insert(i + 1, "")
    return "\n".join(lines)


FIXERS = [
    ("crlf", fix_crlf),
    ("tabs", fix_tabs),
    ("trailing-whitespace", fix_trailing),
    ("keyword-space", fix_keyword_space),
    ("brace-style", fix_brace_style),
    ("header-guard", fix_header_guard),
    ("blank-lines", fix_blank_lines),
    ("eof-newline", fix_eof),
]


# --------------------------------------------------------------------------
# clang-format integration (optional)
# --------------------------------------------------------------------------

CF_WARN_RE = re.compile(r"^(.*?):(\d+):(\d+): (?:warning|error): code should be clang-formatted")


def clang_format_bin(cfg):
    if not cfg["use_clang_format"] or "clang-format" in cfg["disabled_rules"]:
        return None
    return shutil.which(cfg["clang_format_binary"])


def clang_format_check(binary, path, cfg):
    try:
        r = subprocess.run([binary, "--dry-run", "--Werror", "--style=file", path],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return []
    seen, out = set(), []
    for line in r.stderr.splitlines():
        m = CF_WARN_RE.match(line)
        if m and int(m.group(2)) not in seen:
            seen.add(int(m.group(2)))
            out.append(Violation(path, int(m.group(2)), int(m.group(3)), "clang-format",
                                 "Formatting differs from .clang-format", cfg))
    return out


def clang_format_fix(binary, path):
    try:
        subprocess.run([binary, "-i", "--style=file", path], timeout=60,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired):
        pass


# --------------------------------------------------------------------------
# File discovery, config, CLI
# --------------------------------------------------------------------------

def find_repo_root(start):
    d = os.path.abspath(start)
    while True:
        if os.path.exists(os.path.join(d, ".gooselint.json")) or os.path.isdir(os.path.join(d, ".git")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return os.path.abspath(start)
        d = parent


def load_config(path):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    if path and os.path.exists(path):
        with open(path) as f:
            user = json.load(f)
        for k, v in user.items():
            if k.startswith("_"):
                continue
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    unknown = set(cfg["disabled_rules"]) | set(cfg["severity_overrides"])
    unknown -= set(RULES)
    if unknown:
        sys.exit("goose-lint: unknown rule(s) in config: %s" % ", ".join(sorted(unknown)))
    return cfg


def is_excluded(relpath, cfg):
    rp = relpath.replace(os.sep, "/")
    for pat in cfg["exclude"]:
        pat = pat.replace(os.sep, "/")
        if pat.endswith("/"):
            if rp.startswith(pat) or ("/" + pat) in ("/" + rp):
                return True
        elif rp == pat or rp.endswith("/" + pat):
            return True
    return False


def collect_files(paths, root, cfg):
    exts = tuple(cfg["extensions"])
    files = []
    for p in paths:
        if os.path.isfile(p):
            files.append(p)  # explicit files are always linted
            continue
        for dirpath, dirnames, filenames in os.walk(p):
            rel_dir = os.path.relpath(dirpath, root)
            dirnames[:] = sorted(d for d in dirnames
                                 if not is_excluded(os.path.join(rel_dir, d) + "/", cfg))
            for fn in sorted(filenames):
                fp = os.path.join(dirpath, fn)
                if fn.lower().endswith(exts) and not is_excluded(os.path.relpath(fp, root), cfg):
                    files.append(fp)
    return files


def staged_files(root, cfg):
    try:
        r = subprocess.run(["git", "diff", "--cached", "--name-only", "--diff-filter=ACMR"],
                           cwd=root, stdout=subprocess.PIPE, universal_newlines=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        sys.exit("goose-lint: --staged needs to run inside a git repository")
    exts = tuple(cfg["extensions"])
    return [os.path.join(root, f) for f in r.stdout.split()
            if f.lower().endswith(exts) and not is_excluded(f, cfg)]


HOOK = """#!/bin/sh
# Installed by goose_lint.py --install-hook. Bypass once with: git commit --no-verify
exec python3 "$(git rev-parse --show-toplevel)/{script}" --staged
"""


def install_hook(root):
    hooks = os.path.join(root, ".git", "hooks")
    if not os.path.isdir(hooks):
        sys.exit("goose-lint: no .git/hooks directory under %s" % root)
    script = os.path.relpath(os.path.abspath(__file__), root).replace(os.sep, "/")
    dest = os.path.join(hooks, "pre-commit")
    with open(dest, "w") as f:
        f.write(HOOK.format(script=script))
    os.chmod(dest, 0o755)
    print("goose-lint: installed pre-commit hook at %s" % dest)


def read_file(path):
    with open(path, "r", encoding="utf-8", errors="surrogateescape", newline="") as f:
        return f.read()


def write_file(path, text):
    with open(path, "w", encoding="utf-8", errors="surrogateescape", newline="") as f:
        f.write(text)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Style checker/fixer for GOOSE C/C++ code. Reports by default; --fix repairs.")
    ap.add_argument("paths", nargs="*", help="files or directories (default: repo root)")
    ap.add_argument("--fix", action="store_true", help="auto-fix what can be fixed, report the rest")
    ap.add_argument("--staged", action="store_true", help="only lint files staged in git")
    ap.add_argument("--strict", action="store_true", help="treat warnings as errors (non-zero exit)")
    ap.add_argument("--quiet", "-q", action="store_true", help="only print the summary line")
    ap.add_argument("--config", help="path to config JSON (default: <repo>/.gooselint.json)")
    ap.add_argument("--no-clang-format", action="store_true", help="skip clang-format even if installed")
    ap.add_argument("--list-rules", action="store_true", help="list rules and exit")
    ap.add_argument("--install-hook", action="store_true", help="install a git pre-commit hook")
    args = ap.parse_args(argv)

    root = find_repo_root(args.paths[0] if args.paths else os.getcwd())
    cfg = load_config(args.config or os.path.join(root, ".gooselint.json"))
    if args.no_clang_format:
        cfg["use_clang_format"] = False

    if args.list_rules:
        for rid, (sev, fixable, desc) in RULES.items():
            sev = cfg["severity_overrides"].get(rid, sev)
            state = "off" if rid in cfg["disabled_rules"] else sev
            print("%-24s %-8s %-8s %s" % (rid, state, "fixable" if fixable else "manual", desc))
        return 0

    if args.install_hook:
        install_hook(root)
        return 0

    files = staged_files(root, cfg) if args.staged else collect_files(args.paths or [root], root, cfg)
    cf = clang_format_bin(cfg)
    if cfg["use_clang_format"] and cf is None and "clang-format" not in cfg["disabled_rules"]:
        print("goose-lint: note: clang-format not found; skipping layout checks "
              "(install it, or set use_clang_format=false)", file=sys.stderr)

    fixed_files = 0
    violations = []
    for path in files:
        try:
            text = read_file(path)
        except OSError as e:
            print("goose-lint: cannot read %s: %s" % (path, e), file=sys.stderr)
            continue
        if args.fix:
            new = text
            for rule, fixer in FIXERS:
                if rule not in cfg["disabled_rules"]:
                    new = fixer(new, cfg, path)
            if new != text:
                write_file(path, new)
            if cf:
                clang_format_fix(cf, path)
            if read_file(path) != text:
                fixed_files += 1
                if not args.quiet:
                    print("fixed: %s" % os.path.relpath(path))
            text = read_file(path)
        display = os.path.relpath(path)
        vs = lint_text(display, text, cfg)
        if cf:
            vs += [v for v in clang_format_check(cf, path, cfg)
                   if v.rule not in cfg["disabled_rules"]]
            for v in vs:
                v.path = display
        vs.sort(key=lambda v: (v.line, v.col))
        violations.extend(vs)

    errors = [v for v in violations if v.severity == "error"]
    warnings = [v for v in violations if v.severity == "warning"]
    if not args.quiet:
        for v in violations:
            print(v)

    fixable = sum(1 for v in violations if v.fixable)
    summary = "goose-lint: %d file(s) checked, %d error(s), %d warning(s)" % (
        len(files), len(errors), len(warnings))
    if args.fix:
        summary += ", %d file(s) fixed" % fixed_files
    print(summary)
    if violations and not args.fix and fixable:
        print("goose-lint: %d issue(s) can be fixed automatically: rerun with --fix" % fixable)
    if violations and (errors or args.strict):
        manual = len(violations) - (0 if args.fix else fixable)
        if manual:
            print("goose-lint: fix the remaining issues by hand, or suppress a justified one "
                  "with  // goose-lint: disable=<rule>")
    failed = bool(errors) or (args.strict and bool(warnings))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
