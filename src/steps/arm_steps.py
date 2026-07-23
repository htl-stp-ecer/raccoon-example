from raccoon import *

from src.hardware.defs import Defs

# ── Composite arm steps ───────────────────────────────────────────────────────
#
# These functions return a Sequential (a seq([...])), not a single step. Wrapping
# multi-step arm sequences here keeps mission files short and lets the same motion
# be reused across missions without copy-paste.


@dsl  # OPTIONAL — only needed so the WebIDE can discover this step in its palette.
def grab_object() -> Sequential:
    """Lower the arm, open the claw, close onto the object, then lift."""
    return seq([
        # Servo positions are defined in config/servos.yml and become callable
        # steps via codegen. A servo step waits until the move is estimated done.
        parallel(
            Defs.arm_servo.down(),  # waits until the arm reaches "down"
            Defs.claw_servo.open(),
        ),

        # Passing a number = degrees/second → eased ("slow") motion. Here 30°/s so
        # the claw closes gently onto the object instead of snapping shut.
        Defs.claw_servo.closed(30),
        Defs.arm_servo.hold(),  # lift to travel height
    ])


def release_object() -> Sequential:
    """Lower the arm to drop height, open the claw, then raise the arm."""
    return seq([
        Defs.arm_servo.down(),
        Defs.claw_servo.open(),
        Defs.arm_servo.up(),  # no explicit wait needed — the servo step blocks
    ])


# ── Advanced: deferred step ───────────────────────────────────────────────────
#
# Use defer() when the sequence can't be decided until runtime — e.g. it depends
# on a live servo angle or a sensor reading. The factory receives the robot and
# returns the step to run, evaluated at execution time (not when the mission is
# built). Logging here (info/debug) lands in the run log with this file + line.

_ARM_HOLD_DEG = 90  # must match the "hold" position in Defs.arm_servo


def safe_arm_lower() -> Defer:
    """Lower the arm only if it is currently above the hold position.

    Avoids driving the servo into a hard stop when the arm is already at or below
    mid-travel — useful when a previous mission left the arm in an unknown state.
    """
    def _build(_robot):
        current_angle = Defs.arm_servo.get_position()

        if current_angle <= _ARM_HOLD_DEG:
            info(f"Arm already at {current_angle:.1f}° — skipping lower")
            return seq([])  # no-op: arm is already low enough

        info(f"Lowering arm from {current_angle:.1f}° → {_ARM_HOLD_DEG}°")
        return seq([
            Defs.arm_servo.hold(),
        ])

    return defer(_build)
