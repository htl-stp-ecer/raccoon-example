from raccoon import *

from src.hardware.defs import Defs
from src.steps.arm_steps import release_object
from src.steps.delivery_steps import record_delivery


class M03DeliverObjectMission(Mission):
    """Navigate to the delivery zone and deposit the object.

    Demonstrates:
      - Single-sensor line following with follow_line_single()
      - A compound stop condition (distance AND a sensor trigger)
      - Running arm motion in parallel with the final approach
      - Calling a step that talks to a stateful service (record_delivery)
    """

    def sequence(self) -> Sequential:
        return seq([
            # Follow the left edge of the line to the delivery zone. Stop when the
            # right sensor hits the cross-tape marker AND we've covered at least
            # 20 cm (the distance guard avoids a false trigger right at the start).
            follow_line_single(
                Defs.front.left,
                speed=0.8,
                side=LineSide.LEFT,
                kp=0.5,
                kd=0.1,
                until=after_cm(20) & on_black(Defs.front.right),
            ),

            # Approach the drop-off while lowering the arm, so the object is at
            # drop height the moment we stop.
            parallel(
                drive_forward(cm=10),
                Defs.arm_servo.down(),
            ),

            # Return to the heading marked in the previous mission. turn_to_heading_*
            # always takes the SHORTEST path to that absolute heading (the "left"
            # only sets the sign convention, see the docs), so this corrects small
            # drift regardless of which way we drifted.
            turn_to_heading_left(0),

            # Deposit the object, then retract the arm for safe travel.
            release_object(),

            # Tell the delivery tracker service another object landed. The step
            # looks the service up via robot.get_service() and the service logs
            # the running count — see src/service/delivery_tracker_service.py.
            record_delivery(),

            # Drive back to clear the delivery zone.
            drive_backward(cm=20),
            Defs.arm_servo.up(),
        ])
