"""Entry point for ExampleBot.

`raccoon run` uploads the project, regenerates hardware/, then executes
`src.main:main` on the Wombat. `Robot().start()` initialises logging, runs the
setup mission, then each mission in order, and finally the shutdown mission.

Keep this file tiny — all behaviour lives in missions/, steps/, and service/.
"""

import os

from raccoon import info

from src.hardware.robot import Robot


def main():
    # `info()` (and debug/warn/error/trace) is the logging front door. Every call
    # is captured to the per-run JSONL log with the caller's file/line/function.
    # See docs/LOGGING.md for the full picture.
    info(f"ExampleBot starting (verbose={os.getenv('EXAMPLEBOT_VERBOSE') == '1'})")

    robot = Robot()
    robot.start()


if __name__ == "__main__":
    main()
