# Tools

| Script | Purpose |
|---|---|
| `gen_mavlink.py` | Generates the MAVLink C headers in `systems/Comms_Subsystem/mavlink/generated/` from `goose.xml`. The headers are gitignored, so run it once after cloning and after every `goose.xml` change. Run with `uv run tools/gen_mavlink.py` (macOS, Linux, Windows); needs [uv](https://docs.astral.sh/uv/). |
| `comms_bench.py` | Bench test for the radio link (heartbeats, commands, rate changes, signing checks). Not the ground station. |
| `air_sim.py` | Python stand-in for the Teensy comms firmware (dummy telemetry, stub command handler) so `comms_bench.py` can be tested over TCP or a radio without the aircraft. |
| `rf/rtl_power_plot.py` | Waterfall + spectrum plot and burst summary for an `rtl_power` CSV (RTL-SDR band checks). |
| `LINK_TEST_HANDOFF.md` | How the two-machine link test (Windows `air_sim` ↔ Mac `comms_bench`, plus RTL-SDR) is set up, its status, and next steps. |

Details for both are in `systems/Comms_Subsystem/README.md`.
