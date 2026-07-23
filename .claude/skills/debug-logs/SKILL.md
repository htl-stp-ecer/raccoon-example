---
name: debug-logs
description: Pull and analyse the latest RaccoonOS run logs to debug robot behaviour. Use when the user asks why the robot did something, wants to inspect logs, find errors, check timing, or replay a run. Covers libstp.jsonl, cmd_trace, profiling, and localization artifacts.
---

# Debugging a run from the logs

RaccoonOS captures nearly everything a run did to a structured log. Full reference:
`docs/LOGGING.md`. This skill is the fast path.

## 1. Find the newest run

Artifacts land in `.raccoon/downloads/run<N>_<timestamp>/` after `raccoon run`.

```bash
d=$(ls -dt .raccoon/downloads/run*/ | head -1); echo "$d"
```

## 2. Read the main log (`libstp.jsonl`)

Newline-delimited JSON, one record per line. Fields: `t`, `elapsed`, `seq`, `level`,
`file`, `line`, `func`, `msg`. The console only shows `warn`/`error` live — the file
has **everything** (debug/trace included), so the answer is almost always here.

```bash
# Errors and warnings first
grep -E '"level":"(error|warn)"' "$d/libstp.jsonl"

# Everything a specific class/step logged
grep '"func":"DeliveryTrackerService' "$d/libstp.jsonl"

# Pretty-print a slice
tail -30 "$d/libstp.jsonl" | python3 -m json.tool
```

Use `elapsed` (seconds since start) to line up log entries with what the robot was
physically doing at that moment.

## 3. Other artifacts (when relevant)

| Symptom | Artifact | How |
|:--------|:---------|:----|
| what did this run record? | `run.json` | `python3 -m json.tool "$d/run.json"` |
| a mission was slow | `profile.<Mission>.json` | needs `RACCOON_PROFILE=1` (run `--config profile`) |
| motors/servos misbehaving | `cmd_trace.jsonl` | needs `RACCOON_CMD_TRACE=1` — every SPI command |
| path/position looks wrong | `localization.jsonl`, `sensors.mcap` | replay in the localization viewer / Foxglove |

If the needed artifact is missing, re-run with `raccoon run --config profile` (or add
the relevant env flag in `config/run-configurations.yml`) to capture it.

## 4. Add targeted logging and re-run

Nothing is filtered from the file, so add `info`/`debug` around the suspect code:

```python
from raccoon import info
info(f"about to grab: claw={Defs.claw_servo.get_position():.0f}")
```

Then re-run and grep for your message. Inside a `Step`/`RobotService`, use
`self.info(...)` so the record is tagged with the class + method.
