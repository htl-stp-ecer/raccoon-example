from raccoon import *

from src.hardware.defs import Defs
from src.mission_params import MissionParams


class M01NavigateToObjectMission(Mission):
    """Drive from the start position to the object pick-up zone.

    Demonstrates:
      - mark_heading_reference() to lock a compass heading
      - reading a table-side parameter with MissionParams.<name>.get()
      - parallel() to move the arm while driving
      - drive_forward(until=...) with a compound stop condition
    """

    def sequence(self) -> Sequential:
        return seq([
            # Record the current IMU heading as 0°. Every later turn_to_heading_*
            # call is relative to this reference. Absolute heading on this firmware
            # is nearly drift-free — ~6° over a full 2-minute match.
            mark_heading_reference(),

            # Approach while lowering the arm in parallel, so we arrive ready to
            # pick up — no wasted time at the destination. The drive distance comes
            # from the setup-time parameter, so it can be trimmed at the table.
            parallel(
                drive_forward(cm=MissionParams.approach_distance.get()),
                seq([
                    wait_until_distance(7),  # let the drive settle first
                    Defs.arm_servo.hold(),   # arm to horizontal mid-point
                ]),
            ),

            # Turn to face the pick-up zone.
            turn_right(degrees=90),

            # Creep forward until BOTH front sensors see black (the target marker).
            # `&` = AllOf: both conditions must hold at the same instant.
            drive_forward(
                until=on_black(Defs.front.left) & on_black(Defs.front.right)
            ),

            # Back up slightly so the arm can reach the object cleanly.
            drive_backward(cm=3),
        ])
