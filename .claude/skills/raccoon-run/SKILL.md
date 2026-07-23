---
name: raccoon-run
description: Deploy and run ExampleBot on the Wombat with the right run configuration. Use when the user wants to run, deploy, upload, or test the robot, pick a run profile (dev / dev-nc / profile), or connect to the Pi.
---

# Running ExampleBot on the Wombat

`raccoon run` uploads the project, regenerates `src/hardware/` on the device, runs
`src.main`, and pulls logs back. These commands need a **physical Wombat** — if you
are an agent without hardware access, prepare everything and hand the exact command
to the user (they can run it inline with `! <command>`).

## Steps

1. **Check the connection.** Only (re)connect if the target changed or there is no
   active connection:
   ```bash
   raccoon connect <PI_ADDRESS>    # e.g. the pi_address from config/connection.yml
   ```
   Do not reconnect before every run.

2. **Pick the run configuration** (defined in `config/run-configurations.yml`):

   | Goal | Command |
   |:-----|:--------|
   | Full competition run (calibrate, checkpoints, light start) | `raccoon run` |
   | Fast iteration (button start, no checkpoints, still calibrates) | `raccoon run --config dev` |
   | Fastest loop (skip calibration too) | `raccoon run --config dev-nc` |
   | Diagnose motion/timing (cmd trace + profiling + localization) | `raccoon run --config profile` |

3. **After the run**, logs are pulled to `.raccoon/downloads/run<N>_.../`. To analyse
   them, use the `debug-logs` skill.

## Notes

- `raccoon run` respects `.raccoonignore` when uploading.
- If the robot "does nothing", check that `motion_pid` constraints in
  `config/robot.yml` are non-zero and that calibration ran (don't use `dev-nc` for a
  first bring-up).
- Regenerated `src/hardware/defs.py` / `robot.py` reflect the current `config/*.yml`.
  If a device seems wrong, fix the YAML and re-run — never hand-edit the generated files.
