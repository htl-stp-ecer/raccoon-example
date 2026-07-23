from raccoon import *

from src.hardware.defs import Defs


class M99ShutdownMission(Mission):
    """Runs automatically when `shutdown_in` (see robot.py) elapses.

    The shutdown mission's only job is to leave the robot SAFE: motors off,
    servos relaxed. Whatever the robot was doing, it ends up here — so keep it
    short, unconditional, and free of anything that could hang.
    """

    def sequence(self) -> Sequential:
        return seq([
            # Park the arm at a safe travel height BEFORE cutting torque.
            Defs.arm_servo.up(),

            # `fully_disable_servos()` cuts torque to every servo so nothing keeps
            # straining against a hard stop. The runtime also disables all motors
            # on exit via its atexit hook, so the robot ends fully powered down.
            fully_disable_servos(),
        ])
