from libstp import *

from src.hardware.defs import Defs
from src.steps.arm_steps import release_object


class M03DeliverObjectMission(Mission):
    """
    Navigate to the delivery zone and deposit the object.

    Demonstrates:
      - Line following with strafe_follow_line_single()
      - Combining a stop condition from distance AND a sensor trigger
      - Running arm motion in parallel with the final approach
    """

    def sequence(self) -> Sequential:
        return seq([
            # Follow the left-side line to the delivery zone.
            # Stop when the rear sensor sees black (cross-tape marker) AND
            # at least 20 cm have been covered (avoids early false triggers).
            strafe_follow_line_single(
                Defs.front.left,
                speed=0.8,
                side=LineSide.LEFT,
                kp=0.5,
                kd=0.1,
            ).until(after_cm(20) & on_black(Defs.front.right)),

            # Approach the drop-off point while lowering the arm so the
            # object is already at the right height when we stop.
            parallel(
                drive_forward(cm=10),
                Defs.arm_servo.down(),
            ),

            # Deposit the object, then retract the arm for safe travel
            release_object(),

            # Drive back to clear the delivery zone
            drive_backward(cm=20),
            Defs.arm_servo.up(),
        ])
