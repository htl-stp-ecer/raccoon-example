"""Tunable mission parameters, entered on the robot screen during setup.

A ParamSet is how you expose a handful of numbers (approach distances, speeds,
offsets) that you want to tweak at the table without editing code or re-running
codegen. Declare each once as a typed :class:`NumberParam` descriptor:

  * the setup mission ASKS for it   → ``MissionParams.approach_distance.ask("...")``
  * normal mission code READS it    → ``MissionParams.approach_distance.get()``

Always go through the attribute — never a raw string key.

Flags:
  * ``persist=True``  remembers the last value across runs (stored on the robot).
  * ``scoped=True``   keeps a separate value per run-configuration profile, so a
                      ``dev`` run and a ``default`` run can hold different numbers.
"""

from __future__ import annotations

from raccoon import NumberParam, ParamSet


class MissionParams(ParamSet):
    """Values dialled in on the robot's screen at setup time."""

    # How far to drive on the final approach to the object. Handy to trim at the
    # table when the start position shifts by a few centimetres.
    approach_distance = NumberParam(
        default=40.0,
        unit="cm",
        persist=True,
        scoped=True,
    )
