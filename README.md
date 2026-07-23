<div align="center">

<img src="https://raw.githubusercontent.com/htl-stp-ecer/.github/main/profile/raccoon-logo.svg" alt="raccoon-example" width="100"/>

# raccoon-example

**A clean, fully-commented reference robot built with RaccoonOS.**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
![Python](https://img.shields.io/badge/Python-3.13-3776AB?logo=python&logoColor=ffdd54)
![Platform](https://img.shields.io/badge/Platform-KIPR%20Wombat-orange)
![Release](https://img.shields.io/badge/release-v3.0.0-blueviolet)

> 📖 **Full documentation at [raccoon-docs.pages.dev](https://raccoon-docs.pages.dev/)**

</div>

---

Use this as your starting point when reading the documentation or setting up a new
robot. Every file is written to be **readable first** — no competition pressure, no
half-finished experiments, no commented-out blocks. It mirrors the structure and
best practices of the latest competition robots (drumbot, cube-bot).

> **The library is imported as `import raccoon`.** It was formerly called `libstp`;
> if you find `from libstp import ...` in old code, that is outdated — use `raccoon`.

---

## What This Robot Does

ExampleBot is a simple delivery robot:

1. **Setup** — homes the servos, asks for a table-side parameter, calibrates the line
   sensors and drive distance, then waits for the light start.
2. **Navigate** — drives to an object using heading control and a line-sensor stop.
3. **Collect** — picks up the object with an arm and claw, then wall-aligns to reset
   heading error.
4. **Deliver** — follows a line to the drop-off zone, deposits the object (and records
   it via a stateful service), clears the area.
5. **Shutdown** — leaves the robot safe when the match timer elapses.

The task is intentionally simple so the *code structure* is the focus, not the game
logic.

---

## Project Layout

```
raccoon-example/
├── raccoon.project.yml       # Project entry point (name, UUID, format_version, includes)
├── pyproject.toml            # Pinned raccoon-cli / raccoon-library, Python 3.13
├── .raccoonignore            # What NOT to upload to the Wombat
├── CLAUDE.md / AGENTS.md     # Guide for coding agents (Claude Code, Codex, …)
├── docs/
│   ├── ARCHITECTURE.md       # Why the project is shaped this way (layers, codegen)
│   └── LOGGING.md            # How RaccoonOS logging & run artifacts work
├── .claude/skills/           # Project skills: raccoon-run, add-mission, debug-logs
├── config/
│   ├── connection.yml        # SSH / deploy settings
│   ├── hardware.yml          # Sensors + SensorGroup (includes motors.yml + servos.yml)
│   ├── motors.yml            # Motor ports and encoder calibration
│   ├── servos.yml            # Servo ports and named positions
│   ├── missions.yml          # Mission list + roles (setup / shutdown)
│   ├── robot.yml             # Kinematics, PID, odometry, motion constraints
│   └── run-configurations.yml# Named run profiles (default / dev / dev-nc / profile)
└── src/
    ├── main.py               # Tiny entry point
    ├── mission_params.py     # ParamSet — values dialled in on the robot screen
    ├── hardware/
    │   ├── defs.py           # Devices (GENERATED from config/ — here hand-written to read)
    │   └── robot.py          # Assembled robot (GENERATED from config/)
    ├── missions/
    │   ├── m00_setup_mission.py
    │   ├── m01_navigate_to_object_mission.py
    │   ├── m02_collect_object_mission.py
    │   ├── m03_deliver_object_mission.py
    │   └── m99_shutdown_mission.py
    ├── steps/
    │   ├── arm_steps.py       # Composite steps + defer() runtime-decision pattern
    │   └── delivery_steps.py  # Custom Step subclass that calls a service
    └── service/
        └── delivery_tracker_service.py   # RobotService — shared state + logging
```

> In a real project, `defs.py` and `robot.py` are **auto-generated** by `raccoon run`
> from the `config/` YAML. In this example they are written by hand so you can read
> them alongside the YAML and understand exactly what gets generated and why. To make
> a permanent hardware change, **edit the YAML**, not the generated Python.

---

## The Three Layers

RaccoonOS projects separate concerns strictly (see [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)):

| Layer | Directory | Responsibility |
|:------|:----------|:---------------|
| **Mission** | `src/missions/` | *What* to do, in order. Declarative sequencing only. |
| **Step** | `src/steps/` | *One* reusable action. A `@dsl` factory + optional `Step` subclass. |
| **Service** | `src/service/` | Shared, stateful logic via `RobotService`, reached with `robot.get_service(...)`. |

Flow for a new feature: **mission → step → service**.

---

## Concepts Demonstrated

| File | Concepts |
|:-----|:---------|
| `raccoon.project.yml` | `format_version: 2`, config includes, run configurations |
| `config/hardware.yml` | Sensor / motor / servo declarations, `SensorGroup`, `!include-merge` |
| `config/robot.yml` | Kinematics, PID gains, axis constraints, physical geometry |
| `config/run-configurations.yml` | Named run profiles + env flags (`RACCOON_CMD_TRACE`, …) |
| `hardware/defs.py` | `Motor`, `IRSensor`, `SensorGroup`, `ServoPreset` with named positions |
| `hardware/robot.py` | `DifferentialKinematics`, lazy `odometry` property, `UnifiedMotionPidConfig` |
| `mission_params.py` | `ParamSet` / `NumberParam` — table-side tuning (`.ask()` / `.get()`) |
| `m00_setup_mission.py` | setup timer, `run_unless_no_calibrate`, `collect_drive` + `calibration_gate` |
| `m01_navigate…` | `mark_heading_reference()`, `parallel()`, `drive_forward(until=…)`, compound conditions |
| `m02_collect…` | reusable composite step, `wall_align_backward()`, re-marking heading |
| `m03_deliver…` | `follow_line_single()`, `&`/`+`/`|` stop-condition operators, service call |
| `m99_shutdown…` | leaving the robot safe (`fully_disable_servos()`) |
| `steps/arm_steps.py` | `seq()` composition, eased servo moves, `defer()` for runtime decisions |
| `steps/delivery_steps.py` | custom `Step` subclass, `robot.get_service(...)` |
| `service/delivery_tracker_service.py` | `RobotService`, `ClassNameLogger` logging |

---

## Quick Start

```bash
# Clone and enter the project
git clone https://github.com/htl-stp-ecer/raccoon-example.git
cd raccoon-example

# Connect to your Wombat and run
raccoon connect <PI_ADDRESS>
raccoon run
```

See [Getting Started](https://raccoon-docs.pages.dev/00-quick-start) if this is your
first time. For fast iteration use a run profile: `raccoon run --config dev`.

---

## Logging

RaccoonOS captures nearly everything a run did to a structured JSONL log, with exact
source locations and timings. This is the fastest way to debug.

```python
from raccoon import info, debug, warn, error   # log from anywhere
info("Reached pickup zone")
```

After a run, artifacts land in `.raccoon/downloads/run<N>_.../` — `libstp.jsonl` (the
full log), `localization.jsonl`, `profile.<Mission>.json`, `cmd_trace.jsonl`,
`sensors.mcap`, and more. **Full guide: [`docs/LOGGING.md`](docs/LOGGING.md).**

---

## Adapting to Your Robot

### Hardware setup (`raccoon wizard`)

`raccoon wizard` configures your physical hardware — drivetrain type, wheel diameter,
wheelbase, motor ports, sensors, servos — interactively, writing into `config/*.yml`.

### Performance tuning (`raccoon calibrate`)

`raccoon calibrate` runs an automatic characterisation sweep (encoder resolution,
velocity feed-forward, motion constraints) and writes the values into your config.
The in-run calibration step in `m00_setup_mission.py` additionally auto-corrects small
deviations at the start of every run.

### Motion constraints (`config/robot.yml`)

The `linear` / `angular` blocks under `motion_pid` set top speed and braking profile.
**These must not be zero** — a missing or zero `max_velocity` makes the robot sit still
even when commanded to move. `raccoon calibrate` measures and writes these for you.

---

## For Coding Agents

If you use Claude Code (or another agent) on this repo, start with **[`CLAUDE.md`](CLAUDE.md)**
— it encodes the architecture rules, the `raccoon` DSL surface, the run/log workflow,
and how to introspect the library. Three project skills live in `.claude/skills/`:
`raccoon-run`, `add-mission`, and `debug-logs`.

---

## Related Repositories

| Repository | What it is |
|:-----------|:-----------|
| [raccoon-lib](https://github.com/htl-stp-ecer/raccoon-lib) | The core robotics library |
| [raccoon-cli](https://github.com/htl-stp-ecer/raccoon-cli) | `raccoon run`, `raccoon calibrate`, `raccoon wizard` |
| [documentation](https://github.com/htl-stp-ecer/documentation) | Full platform docs |

---

## License

The example code in this repository is MIT licensed — see [LICENSE](LICENSE).

You can freely use, modify, and build on it for your competition robot
**without having to publish your own robot code based on this repo**.
