---
name: add-mission
description: Scaffold a new RaccoonOS mission in this project following the repo's conventions. Use when the user wants to add, create, or insert a new mission (a new match phase / behaviour) into ExampleBot.
---

# Adding a mission

Missions are declarative sequences of steps in `src/missions/`. Follow the existing
conventions exactly so the project stays consistent.

## 1. Create the mission file

Name: `m{nn}_{snake_description}_mission.py`, class `M{nn}{PascalDescription}Mission`.
Numbers order the match; keep `00` = setup and `99` = shutdown.

```python
from raccoon import *

from src.hardware.defs import Defs


class M15PushCubesMission(Mission):
    """One-line summary of what this phase accomplishes.

    Demonstrates: (list the raccoon primitives a reader will learn from here).
    """

    def sequence(self) -> Sequential:
        return seq([
            mark_heading_reference(),
            # ... compose steps: seq / parallel / drive_* / turn_* / follow_line_*
        ])
```

Rules:
- Sequencing only — **no loops or business logic** in the mission. If you need real
  Python at runtime, put it in a step (`src/steps/`); if you need shared state, a
  service (`src/service/`).
- Prefer built-in `raccoon` primitives. Introspect before inventing:
  `RACCOON_PLATFORM=mock python -c "import raccoon; print(inspect...)"`.

## 2. Register it in the mission list

Add it in the correct position in **`config/missions.yml`**:

```yaml
- M15PushCubesMission
```

## 3. Wire it into the robot

Because this repo hand-maintains `src/hardware/robot.py` (normally generated), also:
- add `from src.missions.m15_push_cubes_mission import M15PushCubesMission`
- insert it in the `missions = [ ... ]` list in the right order.

(In a fully generated project, `raccoon run` does this from `missions.yml` for you.)

## 4. Verify it builds

```bash
RACCOON_PLATFORM=mock python -c "from src.hardware.robot import Robot; \
  [m.sequence() for m in Robot().missions]; print('OK')"
```

This catches DSL/import errors without a Wombat. Then run on hardware with the
`raccoon-run` skill.
