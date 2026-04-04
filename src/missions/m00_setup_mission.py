from libstp import *

from src.hardware.defs import Defs


class M00SetupMission(SetupMission):
    """
    Runs once before the match.

    The setup mission is the right place for:
      - Moving servos to their starting positions
      - Running the IR sensor calibration (black / white thresholds)
      - Waiting for the light-start signal or a button press
    """

    def sequence(self) -> Sequential:
        return seq([
            # Move arm and claw to known starting positions
            Defs.arm_servo.up(),
            Defs.claw_servo.open(),

            # Calibrate IR sensors: the robot first sees black tape, then
            # white surface. distance_cm controls how far it drives for each.
            calibrate(distance_cm=50),

            # Wait for the human to press the button -> Handled automatically
            #- wait for light if light sensor is available
            #- wait for button if dev mode or no WFL sensor found
        ])
