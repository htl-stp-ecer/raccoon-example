# How RaccoonOS logging works

RaccoonOS has a single, structured logging pipeline. Understanding it is the
fastest way to debug a run: almost everything the robot did is in the log, with
exact source locations and timestamps. This page explains **how to log**, **where
logs go**, and **what every run artifact is for**.

---

## 1. Writing log messages

From anywhere in mission/step code:

```python
from raccoon import info, debug, warn, error, trace

info("Reached pickup zone")
debug(f"heading={heading:.1f}° distance={dist:.2f}m")
warn("Retrying grab — claw did not close")
error("Vision daemon unreachable")
```

Inside a `Step`, `UIStep`, or `RobotService`, use the `self.` variants instead —
they additionally tag the record with the class and method:

```python
class DeliveryTrackerService(RobotService):
    def record(self) -> None:
        self.info("Delivered an object")   # func field → "DeliveryTrackerService.record"
```

### Levels

| Level    | Use for                                   | Console? | JSONL file? |
|:---------|:------------------------------------------|:--------:|:-----------:|
| `trace`  | very fine-grained, per-cycle detail       |    no    |     yes     |
| `debug`  | routine diagnostic detail                 |    no    |     yes     |
| `info`   | milestones / normal progress              |    no    |     yes     |
| `warn`   | recoverable problems, retries             |  **yes** |     yes     |
| `error`  | failures                                  |  **yes** |     yes     |

Two things that surprise newcomers:

1. **There is no runtime level filtering.** Every compiled-in call is captured to
   the file — `debug`/`trace` included. So log freely; it won't be silently dropped.
2. **The console is deliberately quiet.** Only `warn`/`error` print live on the
   robot screen / terminal. For everything else, read the JSONL file after the run.

---

## 2. What a log record contains

The primary log file is **newline-delimited JSON** (`libstp.jsonl`) — one JSON
object per line. Each record captures the caller's source location automatically:

```json
{"t":"2026-07-12T03:10:15.642","elapsed":0.016,"seq":1,"level":"info",
 "logger":"core","thread":987,"pid":987,
 "file":".../raccoon/__init__.py","line":356,"func":"_log_startup_banner",
 "msg":"raccoon v1.0.0 | raccoon-transport v0.1.66"}
```

| Field     | Meaning |
|:----------|:--------|
| `t`       | wall-clock timestamp (ISO 8601, millisecond precision) |
| `elapsed` | seconds since the program started — great for timing |
| `seq`     | monotonic sequence number (stable ordering even at equal `t`) |
| `level`   | `trace` / `debug` / `info` / `warn` / `error` |
| `logger`  | `core`, or the subsystem name |
| `thread` / `pid` | source thread and process |
| `file` / `line` / `func` | exact caller location; `func` is `Class.method` when logged via `self.` |
| `msg`     | your message string |

Because `file`/`line`/`func` are discrete fields, you can pinpoint *which line*
emitted any message — no need to grep source for a string.

There is also a human-readable mirror, `.raccoon/logs/libstp.log`, formatted as:

```
2026-07-03 14:08:20 | 0.001s | info | h.p...r.api.py | [M000SetupMission]: Starting mission
        │                │        │         │                  │
    timestamp        elapsed    level    source module     [Component]: message
```

Steps log their own progress here automatically, e.g.
`[PauseSetupTimer]: 1/17: PauseSetupTimer()` and `Finished ... in 0.000s`, so you
can see the mission walk through its sequence step by step.

---

## 3. Where logs live

On the **Wombat**, each run gets its own directory:

```
/home/pi/programs/<project-uuid>/.raccoon/runs/<run_id>/
```

After the run, `raccoon run` **pulls the artifacts back** to your machine under:

```
.raccoon/downloads/run<N>_<YYYYMMDD-HHMMSS>/
```

(The `.raccoon/` directory is git-ignored — it's local run state, not source.)
Pass `no_sync` in a run configuration to skip the pull-back.

---

## 4. Run artifacts — the full set

Every entry below appears in a `.raccoon/downloads/run<N>_.../` folder. Which ones
are populated depends on the run configuration (see
`config/run-configurations.yml`).

| File | What it is | Enabled by |
|:-----|:-----------|:-----------|
| `libstp.jsonl` | **The main structured log.** Everything, all levels. Start here. | always |
| `run.json` | Run metadata: id, start/end time, missions, which artifacts were recorded | always |
| `manifest.json` | Index of artifacts in this run folder + their sizes | always |
| `localization.jsonl` | Recorded robot pose over time — replay the path offline | `record_localization` |
| `profile.<Mission>.json` | Per-mission timing/profiling breakdown | `RACCOON_PROFILE=1` |
| `cmd_trace.jsonl` | Every low-level SPI command sent to the STM32 (servo/motor writes) | `RACCOON_CMD_TRACE=1` |
| `sensors.mcap` | Sensor stream in [MCAP](https://mcap.dev) format — open in Foxglove | sensor recording |
| `journal.<service>.jsonl` | systemd journal for each background daemon service | when daemons run |

### Reading them

```bash
# The newest run folder
d=$(ls -dt .raccoon/downloads/run*/ | head -1)

# All errors and warnings from the last run
grep -E '"level":"(error|warn)"' "$d/libstp.jsonl"

# Everything a specific class logged
grep '"func":"DeliveryTrackerService' "$d/libstp.jsonl"

# Pretty-print the last 20 records
tail -20 "$d/libstp.jsonl" | python3 -m json.tool

# What did this run record?
python3 -m json.tool "$d/run.json"
```

`sensors.mcap` and `localization.jsonl` are for visual replay (Foxglove / the
localization viewer), not for grepping.

---

## 5. Turning extra logging on

Run configurations (in `config/run-configurations.yml`) set the environment
variables that control optional artifacts:

```yaml
profile:
  record_localization: true      # → localization.jsonl
  env:
    RACCOON_CMD_TRACE: '1'        # → cmd_trace.jsonl
    RACCOON_PROFILE: '1'          # → profile.<Mission>.json
```

Then: `raccoon run --config profile`.

---

## 6. Debugging checklist

1. Reproduce the run, then open the newest `.raccoon/downloads/run*/libstp.jsonl`.
2. `grep '"level":"error"'` — start at the first error and read `file`/`line`.
3. Use `elapsed` to correlate log lines with what the robot was physically doing.
4. Add your own `info`/`debug` around the suspect step and run again — nothing is
   filtered out, so it will be in the file.
5. For motion issues, enable `RACCOON_CMD_TRACE` and inspect `cmd_trace.jsonl`, or
   record localization and replay the path.
