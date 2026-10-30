# ExampleBot architecture

This page explains *why* the project is shaped the way it is. Read it once and the
file layout will feel obvious.

---

## The big picture

```
config/*.yml  ──(raccoon run: codegen)──▶  src/hardware/defs.py + robot.py
                                                     │
                                                     ▼
   main.py ──▶ Robot().start() ──▶ setup ▶ mission ▶ mission ▶ … ▶ shutdown
                                     │        │
                                     │        └── seq / parallel of STEPS
                                     │                     │
                                     │                     └── some steps call a SERVICE
                                     └── calibration, homing, params
```

- **YAML → generated Python.** You describe hardware in `config/*.yml`. `raccoon run`
  generates `defs.py` (every device as a `Defs.<name>`) and `robot.py` (the assembled
  robot: drivetrain, PID, geometry, mission list). *They are regenerated on every run
  and are git-ignored, so they are not in a fresh checkout — edit the YAML, never the
  generated Python. Run `raccoon codegen` to produce them without a robot.*
- **The runtime drives the match.** `Robot().start()` runs the setup mission once,
  then each mission in order, then the shutdown mission when `shutdown_in` elapses.

---

## The three layers

### 1. Missions — *what to do* (`src/missions/`)

A mission is a declarative sequence. It reads like a plan, top to bottom, and
contains **no business logic or loops** — just composition of steps.

```python
class M010NavigateToObjectMission(Mission):
    def sequence(self) -> Sequential:
        return seq([
            mark_heading_reference(),
            parallel(drive_forward(cm=40), Defs.arm_servo.hold()),
            turn_right(degrees=90),
            drive_forward(until=on_black(Defs.front.left) & on_black(Defs.front.right)),
        ])
```

One file per mission, numbered with three digits: `m000` (setup), `m010`–`m998` (main missions), `m999` (shutdown). The order and
roles are declared in `config/missions.yml`; codegen turns that list into the generated
`robot.py`.

### 2. Steps — *one reusable action* (`src/steps/`)

A step is the unit of composition. Two flavours:

- **Composite function** — returns `seq([...])`, groups a few primitives so missions
  can reuse them (`grab_object`, `release_object` in `arm_steps.py`).
- **Custom `Step` subclass** — when you need real Python at execution time: subclass
  `Step` (or `UIStep` for screen interaction), implement async `_execute_step(robot)`,
  and expose it via a small `@dsl` factory (`record_delivery` in `delivery_steps.py`).

The `@dsl` decorator registers the factory so the WebIDE can offer it in its palette;
it's optional for code-only projects but good practice.

### 3. Services — *shared, stateful logic* (`src/service/`)

Some state must outlive a single step and be shared across the match — a counter, a
calibration model, a controller. That belongs in a `RobotService`. Steps reach it
through `robot.get_service(MyService)`, which creates the instance on first use and
caches it:

```python
class RecordDeliveryStep(Step):
    async def _execute_step(self, robot):
        robot.get_service(DeliveryTrackerService).record()
```

`RobotService` extends `ClassNameLogger`, so `self.info(...)` inside a service is
automatically attributed to the class + method in the logs.

---

## Why the strict split?

- **Readable missions.** A mission you can read like prose is a mission you can debug
  under competition pressure. Loops and algorithms hidden in a mission are where bugs
  live.
- **Reuse.** The same `grab_object` step serves any mission. The same service holds
  state no matter which step touches it.
- **Regeneration-safe.** Hardware lives in YAML, so regenerating `defs.py`/`robot.py`
  never clobbers your logic — your logic isn't in those files.

---

## `mission_params.py` — table-side tuning

Values you want to trim at the competition table (approach distances, offsets)
without editing code go in a `ParamSet`. Declared once as typed descriptors, asked
for in setup (`.ask(...)`), read in missions (`.get()`), optionally persisted across
runs and scoped per run configuration. See `src/mission_params.py`.

---

## Where things go — quick reference

| I want to…                                    | Edit |
|:----------------------------------------------|:-----|
| change a port / servo angle / PID gain        | `config/*.yml`, then `raccoon run` |
| change the order missions run in              | `config/missions.yml` (codegen updates `robot.py`) |
| change what a mission does                     | `src/missions/m*.py` |
| add a reusable action                          | `src/steps/*.py` |
| hold state across steps / a control algorithm  | `src/service/*.py` |
| expose a table-side tunable number             | `src/mission_params.py` |
| add a named run profile / env flags            | `config/run-configurations.yml` |
