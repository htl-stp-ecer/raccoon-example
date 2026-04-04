from libstp import *

from src.hardware.defs import Defs

# ── Composite arm steps ───────────────────────────────────────────────────────
#
# These functions return a Sequential (a seq([...])) rather than a single step.
# Wrapping multi-step arm sequences here keeps mission files concise and
# lets the same sequence be reused across multiple missions without duplication.


def grab_object() -> Sequential:
    """Lower the arm, open the claw, close onto the object, then lift."""
    return seq([
        Defs.arm_servo.down(),
        Defs.claw_servo.open(),
        wait_for_seconds(0.2),   # settle before gripping
        Defs.claw_servo.closed(),
        wait_for_seconds(0.1),   # let the claw lock
        Defs.arm_servo.hold(),   # lift to travel height
    ])


def release_object() -> Sequential:
    """Lower the arm to drop height, open the claw, then raise the arm."""
    return seq([
        Defs.arm_servo.down(),
        Defs.claw_servo.open(),
        wait_for_seconds(0.2),   # give the object time to fall clear
        Defs.arm_servo.up(),
    ])


# ── Advanced: deferred step ───────────────────────────────────────────────────
#
# Use defer() when the step sequence cannot be determined until runtime —
# for example, when it depends on the current servo angle or a sensor value.
#
# The factory function receives the robot instance and returns a step.
# This is identical to the pattern used in drum_lifting_step.py.

_ARM_HOLD_DEG = 90   # must match the "hold" value in Defs.arm_servo


def safe_arm_lower() -> Defer:
    """
    Lower the arm only if it is currently above the hold position.

    This avoids driving the servo into a hard stop when the arm is already
    at or below mid-travel — useful when the previous mission may have left
    the arm in an uncertain state.
    """
    def _build(_):
        current_angle = Defs.arm_servo.get_position()

        if current_angle <= _ARM_HOLD_DEG:
            info(f"Arm already at {current_angle:.1f}° — skipping lower")
            return seq([])                    # no-op: arm is already low enough

        info(f"Lowering arm from {current_angle:.1f}° → {_ARM_HOLD_DEG}°")
        return seq([
            Defs.arm_servo.hold(),
            wait_for_seconds(0.1),
        ])

    return defer(_build)
