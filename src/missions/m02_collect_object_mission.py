from raccoon import *

from src.hardware.defs import Defs
from src.steps.arm_steps import grab_object


class M02CollectObjectMission(Mission):
    """Pick up the object and prepare for delivery.

    Demonstrates:
      - Calling a reusable composite step (grab_object) from steps/arm_steps.py
      - wall_align_backward() to physically square up against a surface
      - re-marking the heading after alignment to erase accumulated error
    """

    def sequence(self) -> Sequential:
        return seq([
            # Reusable arm sequence. Keeping it in steps/ keeps the mission readable
            # and lets other missions reuse the exact same pick-up motion.
            grab_object(),

            # Drive backwards into the wall and stop the instant the accelerometer
            # spikes (impact). This physically squares the chassis to the wall.
            # accel_threshold depends on robot weight — tune it per robot.
            wall_align_backward(accel_threshold=0.3),

            # Now that we are physically square, redefine "0°" here. This erases
            # any heading drift picked up during the grab.
            mark_heading_reference(),

            # Turn to face the delivery zone.
            turn_right(degrees=90),
        ])
