# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pymavlink==2.4.50",  # keep in step with the ground station's pymavlink
#     "lxml",
# ]
# ///
"""Generate the MAVLink C headers in systems/Comms_Subsystem/mavlink/generated/
from systems/Comms_Subsystem/mavlink/goose.xml.

Works the same on macOS, Linux and Windows. Needs uv (https://docs.astral.sh/uv/),
which reads the dependency block above and fetches pymavlink itself. From the repo root:

    uv run tools/gen_mavlink.py

The generated headers are gitignored, so run this once after cloning and again after
every change to goose.xml (yours or pulled). Never edit them by hand. When goose.xml
changes, send the new file to the ground-station author.
"""
import shutil
import sys
import tempfile
from pathlib import Path

import pymavlink
from pymavlink.generator import mavgen, mavparse

MAV_DIR = Path(__file__).resolve().parent.parent / "systems" / "Comms_Subsystem" / "mavlink"
OUT_DIR = MAV_DIR / "generated"


def main() -> int:
    # goose.xml includes common.xml (-> standard.xml -> minimal.xml). Those are vendored
    # next to goose.xml, copied from pymavlink 2.4.50, because some pymavlink installs
    # (e.g. Linux wheels) ship without message_definitions/.
    with tempfile.TemporaryDirectory(prefix="goose_mavgen_") as work:
        # Generate into the scratch folder first, so a failed run leaves the old headers.
        new_out = Path(work) / "generated"
        opts = mavgen.Opts(str(new_out), wire_protocol=mavparse.PROTOCOL_2_0, language="C")
        if not mavgen.mavgen(opts, [str(MAV_DIR / "goose.xml")]):
            print("mavgen failed; existing headers left unchanged", file=sys.stderr)
            return 1

        shutil.rmtree(OUT_DIR, ignore_errors=True)
        shutil.copytree(new_out, OUT_DIR)

    print(f"Generated {OUT_DIR} with pymavlink {pymavlink.__version__}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
