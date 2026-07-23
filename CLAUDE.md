# CLAUDE.md — Agent guide for ExampleBot (RaccoonOS)

Guidance for Claude Code (and any coding agent) working in this repository. Humans
should read [`README.md`](README.md) first; this file is the machine-facing contract.

---

## What this project is

ExampleBot is the **reference robot** for RaccoonOS: a clean, fully-commented
project that demonstrates every core concept once. It is a KIPR Wombat robot
programmed in Python against the `raccoon` library. The task (navigate → collect →
deliver an object) is deliberately trivial so the *structure* is the lesson.

> The library is imported as `import raccoon`. It used to be called `libstp`; any
> `from libstp import ...` you see anywhere is outdated — always use `raccoon`.

---

## Source of truth

- **`raccoon.project.yml` + `config/*.yml` are the source of truth for hardware.**
- **`src/hardware/defs.py` and `src/hardware/robot.py` are GENERATED artifacts.**
  On a real project `raccoon run` regenerates them from the YAML on every run.
  In *this* repo they are hand-written and commented for teaching, but the rule
  still holds: to change a port, servo position, PID gain, or kinematic value,
  **edit the YAML**, not the Python. Hand edits to generated files get overwritten.

---

## Code structure — where new logic goes

```
src/
├── main.py            # tiny entry point — do not grow this
├── mission_params.py  # ParamSet: values dialled in on the robot screen at setup
├── hardware/          # GENERATED — defs.py (devices) + robot.py (assembled robot)
├── missions/          # HIGH-LEVEL sequencing only. One file per mission.
├── steps/             # reusable actions; custom Step subclasses; @dsl factories
└── service/           # stateful, shared logic (RobotService) used by steps
```

The layering is strict — respect it:

| Layer       | Holds                              | Rule |
|:------------|:-----------------------------------|:-----|
| **mission** | *what* to do, in order             | sequencing only — no business logic, no long control loops |
| **step**    | *one* reusable action / boundary   | `@dsl` factory + optional `Step`/`UIStep` subclass |
| **service** | shared state & control algorithms  | `RobotService`, looked up via `robot.get_service(...)` |

Flow for a new mechanism feature: **mission → step → service**. Put the sequence
in a mission, the action boundary in a step, the stateful mechanism logic in a
service.

---

## Implementation rules

- Add/adjust match flow in `src/missions/*`. Keep missions readable — if a mission
  grows a loop or algorithm, push it down into a step or service.
- Reusable actions go in `src/steps/*`. A step is either a plain function returning
  `seq([...])`, or a `Step`/`UIStep` subclass with an async `_execute_step`.
  Expose it to missions through a small `@dsl` factory function.
- Shared, longer-lived state goes in a `RobotService` under `src/service/*`, reached
  with `robot.get_service(MyService)` (lazily created, then cached).
- Hardware wiring / ports / kinematics stay in `config/*.yml`; regenerate via
  `raccoon run`.
- **Prefer `raccoon` primitives over custom code.** The library already provides a
  huge amount (drive, turns, line following, lineups, wall aligns, calibration,
  conditions). Check what exists before writing your own. Default strategy: use
  built-ins; write custom logic only for genuinely missing capabilities.

---

## The `raccoon` DSL in one screen

Missions do `from raccoon import *` and compose steps:

- **Composition**: `seq([...])` (in order), `parallel(...)` (all at once),
  `background(step, name="x")` + `wait_for_background(name="x")`,
  `optimize([...])` (like `seq` but lets the planner blend adjacent moves),
  `timeout_or(step, seconds=3, fallback=...)`.
- **Drive**: `drive_forward(cm=, speed=, heading=, until=)`, `drive_backward(...)`,
  `turn_right(degrees=)`, `turn_left(...)`, `turn_to_heading_left(deg)` /
  `turn_to_heading_right(deg)` (absolute; always shortest path),
  `strafe_left/right(...)` (mecanum only), `wall_align_backward(accel_threshold=)`.
- **Heading**: `mark_heading_reference()` sets "0°" at the current pose; every
  `turn_to_heading_*` is relative to the last reference. Heading is near-drift-free.
- **Line work**: `follow_line_single(sensor, side=LineSide.LEFT, kp=, kd=, until=)`,
  the fluent `line_follow().single(...).move(...).pid(...)`, `lineup(...)`.
- **Stop conditions** (pass to `until=` or combine with operators):
  `on_black(sensor)`, `on_white(sensor)`, `over_line(sensor)`, `after_cm(n)`,
  `after_seconds(n)`, `wait_until_distance(cm)`.
  - `a + b` → **then** (b starts being checked after a fires) — sequential.
  - `a & b` → **all of** (both true at the same instant).
  - `a | b` → **any of** (either true).
- **Setup/calibration**: `pause_setup_timer()` / `start_setup_timer()`,
  `run_unless_no_calibrate(...)`, `collect_drive(collect_ir_set(drive, set_name=, sensors=))`,
  `calibration_gate(require_axes=[CalibrationAxis.FORWARD], require_ir_sets=[...])`,
  `wait_for_button("msg")`.
- **Servos**: named positions from YAML become steps — `Defs.arm_servo.down()`,
  `Defs.arm_servo.down(speed=120)` (eased, °/s), `Defs.arm_servo.up(blocking=False)`.
- **Params**: `MissionParams.<name>.ask("prompt")` in setup, `.get()` in missions.

Not sure a symbol exists? It's re-exported from `raccoon`. Verify with
`python -c "import raccoon; print(hasattr(raccoon, 'NAME'))"` (see next section).

---

## Workflow — running the robot

- **Connect** only when the target changed or there is no active connection:
  `raccoon connect <PI_ADDRESS>`. Do NOT reconnect before every run.
- **Run** with `raccoon run` (optionally `--config dev` / `dev-nc` / `profile` — see
  `config/run-configurations.yml`). `raccoon run` will:
  1. upload the local project to the Wombat (respecting `.raccoonignore`),
  2. run codegen on the device (regenerating `hardware/`),
  3. execute `src.main`,
  4. pull run artifacts (logs) back so local and remote stay in sync.
- **Hardware setup**: `raccoon wizard` (interactive) writes drivetrain/ports/sensors
  into `config/*.yml`.
- **Calibration**: `raccoon calibrate` characterises the drivetrain and writes
  motion constraints / feed-forward into the config. The in-run calibration step in
  `m00_setup_mission.py` additionally auto-corrects small deviations each run.

You (the agent) generally **cannot** run these — they need a physical Wombat. Make
the code changes and tell the human which command to run.

---

## Logging — how to observe what happened

Full reference: [`docs/LOGGING.md`](docs/LOGGING.md). The essentials:

- Log from anywhere: `from raccoon import info, debug, warn, error, trace`. Inside a
  `RobotService`/`Step` use `self.info(...)` etc. (tags the class + method).
- **Every** call is captured to the per-run JSONL log with file/line/function.
  The console shows only `warn`/`error`; the JSONL file captures everything.
- After a run, artifacts land in `.raccoon/downloads/run<N>_<timestamp>/`:
  `libstp.jsonl` (full log), `localization.jsonl`, `profile.<Mission>.json`,
  `cmd_trace.jsonl`, `sensors.mcap`, `journal.<service>.jsonl`, `run.json`.
- To debug a run, read the newest `libstp.jsonl`. It is newline-delimited JSON —
  grep by `"level":"error"` or a `func`/`msg` substring.

---

## Discovering `raccoon` APIs (when unsure)

Do this before inventing an unsupported call:

```bash
# What's exported? (mock platform so it runs on a laptop)
RACCOON_PLATFORM=mock python -c "import raccoon; print([n for n in dir(raccoon) if 'line' in n.lower()])"

# Signature of a specific helper
RACCOON_PLATFORM=mock python -c "import raccoon, inspect; print(inspect.signature(raccoon.follow_line_single))"
```

On the Pi you can also SSH in and introspect the installed `raccoon` (`dir()`,
`help()`, `inspect`). Prefer discovery over guessing.

---

## Conventions

- Missions: `M{nn}{Description}Mission` in `m{nn}_{description}_mission.py`
  (`M01NavigateToObjectMission`). `00` = setup, `99` = shutdown.
- Steps: verb-first `@dsl` factory (`grab_object`, `record_delivery`); `Step`
  subclasses `PascalCaseStep`.
- Services: `PascalCaseService(RobotService)`, one responsibility each.
- Keep the didactic, comment-rich style of this repo — it is a teaching reference.

---

## Do / Don't

- ✅ Edit YAML for hardware; regenerate with `raccoon run`.
- ✅ Keep missions declarative; push logic into steps/services.
- ✅ Reuse `raccoon` primitives; introspect before inventing.
- ✅ Log with `info/debug/...`; read `libstp.jsonl` to debug.
- ❌ Don't hand-edit `hardware/defs.py` / `robot.py` for permanent changes.
- ❌ Don't `from libstp import ...` — the library is `raccoon`.
- ❌ Don't put control loops or shared state directly in a mission file.
- ❌ Don't reconnect/recalibrate on every run when it isn't needed.
