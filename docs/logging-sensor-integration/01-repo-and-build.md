# Part 1: Repo and build fixes

[← Overview](README.md)

Git was never the problem. Four libraries committed as empty folders, plus a stray `lib_dir` line, meant PlatformIO saw two copies of every sensor library and one of them was empty.

## Git was fine

The reflog showed that the `git reset --hard origin/logging_subsystem` landed on the same commit a pull would have, and nothing was lost:

```
9f3b6ca HEAD@{0}: pull origin logging_subsystem: Fast-forward
9cae816 HEAD@{1}: reset: moving to origin/logging_subsystem
9cae816 HEAD@{2}: pull: Fast-forward
```

The branch history, newest first:

| Commit | Author | Message |
| --- | --- | --- |
| 9f3b6ca | Garcia | Fixed missing return |
| 379a14a | Garcia | Fixed header gaurds |
| 9cae816 | lightningstar425 | fix sensor bugs |
| 31f2b3f | lightningstar425 | need to add virtual/overide to sensor classes |
| 693feac | lightningstar425 | merged logging and sensors (has library linking issues) |
| bf53171 | Advik Sharma | Count bytes lost to failed SD writes |

Useful habit: when something looks lost, `git reflog` before anything else. It lists every place `HEAD` has been, including resets.

## Empty submodule pointers in lib/

`git diff --stat` showed four suspicious one-line entries:

```
 lib/SparkFun_LSM9DS1_Arduino_Library  |   1 +
 lib/bmp390                            |   1 +
 lib/ms4525do                          |   1 +
 lib/ublox                             |   1 +
```

`git ls-tree` showed what they really were: mode `160000`, a **gitlink**. That's a pointer to a commit in another repository, which is how git stores a submodule.

```
160000 commit 18d3181c... lib/SparkFun_LSM9DS1_Arduino_Library
```

Someone had cloned the libraries into `lib/` with `git clone` and then ran `git add lib/`. Git doesn't copy a folder that is itself a repository. It records a pointer to that repository's current commit. There was no `.gitmodules` file, so nobody else's checkout knows where to fetch them from, and everyone got four empty folders.

**Fix:** `git rm --cached` on the four pointers and delete the empty folders. The libraries were already listed in `lib_deps`, which is the right place for them. PlatformIO downloads them into `.pio/libdeps/` itself.

## lib_dir and the double source

The merge also added this to `platformio.ini`:

```ini
[platformio]
lib_dir = lib
```

`lib` is already the default, so the line did nothing by itself. But with the empty folders present, PlatformIO's library finder had two candidates for each library: an empty local `lib/bmp390` and the real one from `lib_deps`. That's where the "library linking issues" came from. The extra `-I $PROJECT_LIBDEPS_DIR/.../bmp390/driver` and `/example` include paths were someone working around it. `driver/` doesn't exist in that library (its code is in `src/`), and nothing needs `example/`.

**Fix:** remove `lib_dir`, keep only the `bmp390/interface` include path. That one is real: libdriver keeps the interface header there, outside `src/`.

## The sensors exclusion

The Teensy build used to exclude sensors on purpose, because they had never compiled:

```ini
; systems/sensors is not built yet: it has never compiled (see systems/README.MD).
; Drop the -<../systems/sensors/> line once it does.
build_src_filter =
    +<*>
    +<../systems/>
    ; -<../systems/sensors/>
```

The merge commented out the exclusion and left the comment, so the file said one thing and did another. Since the teammates clearly meant to build sensors, they stay in the build, and the comment now describes reality.

## The environments now

| Env | Defines | Sensors | Use it to |
| --- | --- | --- | --- |
| `teensy41` | `GNC_HARDWARE_BUILD` | real drivers | fly / bench-test with hardware |
| `teensy41_dummy` | `GNC_HARDWARE_BUILD`, `GOOSE_DUMMY_SENSORS` | dummy | test SD logging on a bare Teensy |
| `native` | none | dummy | `pio test -e native` on your laptop |

`teensy41_dummy` uses `extends = env:teensy41` and appends one flag, so it can never drift from the real build. [Part 4](04-native-vs-hardware.md) explains how the define picks the sensors.
