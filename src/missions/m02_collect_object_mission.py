from libstp import *

from src.hardware.defs import Defs
from src.steps.arm_steps import grab_object


class M02CollectObjectMission(Mission):
    """
    Pick up the object and prepare for delivery.

    Demonstrates:
      - Calling a reusable composite step (grab_object)
      - wall_align_backward() to square up against a surface
      - turn_to_heading() to restore a precise heading after alignment
    """

    def sequence(self) -> Sequential:
        return seq([
            # Use the reusable grab_object step defined in steps/arm_steps.py.
            # Encapsulating the arm sequence there keeps the mission readable
            # and lets other missions reuse the same logic.
            grab_object(),

            # Square up against the back wall before navigating to delivery.
            # This resets any heading error accumulated during collection.
            wall_align_backward(speed=0.5, accel_threshold=0.3),
            mark_heading_reference(),

            # Turn to face the delivery zone
            turn_right(degrees=90),
        ])
