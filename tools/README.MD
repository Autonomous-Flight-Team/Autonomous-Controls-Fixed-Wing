# Tools

| Script | Purpose |
|---|---|
| `gen_mavlink.py` | Generates the MAVLink C headers in `systems/Comms_Subsystem/mavlink/generated/` from `goose.xml`. The headers are gitignored, so run it once after cloning and after every `goose.xml` change. Run with `uv run tools/gen_mavlink.py` (macOS, Linux, Windows); needs [uv](https://docs.astral.sh/uv/). |
| `comms_bench.py` | Bench test for the radio link (heartbeats, commands, rate changes, signing checks). Not the ground station. |

Details for both are in `systems/Comms_Subsystem/README.md`.
