from libstp import *

from src.hardware.defs import Defs


class M01NavigateToObjectMission(Mission):
    """
    Drive from the start position to the object pick-up zone.

    Demonstrates:
      - mark_heading_reference() to lock a compass heading
      - turn_right() / drive_forward() for basic manoeuvring
      - drive_forward().until() with a sensor-based stop condition
      - parallel() to move the arm while driving
    """

    def sequence(self) -> Sequential:
        return seq([
            # Record the current IMU heading as 0°. All subsequent
            # turn_to_heading() calls are relative to this reference.
            mark_heading_reference(),

            # Approach the line while lowering the arm in parallel so we
            # arrive ready to pick up — no wasted time at the destination.
            parallel(
                drive_forward(cm=40),
                seq([
                    wait_for_seconds(0.5),       # let the drive settle first
                    Defs.arm_servo.hold(),        # arm to horizontal mid-point
                ]),
            ),

            # Turn to face the pick-up zone
            turn_right(degrees=90),

            # Follow the line until both front sensors see black,
            # meaning we have reached the target marker.
            drive_forward().until(
                on_black(Defs.front.left) & on_black(Defs.front.right)
            ),

            # Back up slightly so the arm can reach the object cleanly
            drive_backward(cm=3),
        ])
