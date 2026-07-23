"""Hardware definitions for ExampleBot.

NORMALLY GENERATED. On a real project `raccoon run` regenerates this file from
config/hardware.yml + config/motors.yml + config/servos.yml every time you run.
Here it is written by hand and heavily commented so you can read it side by side
with the YAML and see exactly what codegen produces (and why).

Rule of thumb: edit the YAML, not this file. Any hand edits here are overwritten
on the next `raccoon run`. The one exception is while you are reading/learning —
which is the whole point of this repo.

Note the imports: everything comes from `raccoon` (the library was formerly
called `libstp`). The generator imports from precise submodules; `import raccoon`
re-exports all of these names too, so mission code can just do
`from raccoon import *`.
"""

from raccoon.foundation import MotorCalibration
from raccoon.hal import AnalogSensor, DigitalSensor, IMU, Motor, Servo
from raccoon.sensor_ir import IRSensor
from raccoon.step.motion.sensor_group import SensorGroup
from raccoon.step.servo.preset import ServoPreset


class Defs:
    # ── Inertial measurement unit ─────────────────────────────────────────────
    imu = IMU()

    # ── Start button (port 10 is the standard Wombat button) ──────────────────
    button = DigitalSensor(port=10)

    # ── Light-start sensor ─────────────────────────────────────────────────────
    # Analog reflectance sensor pointed at the start light. The setup mission's
    # light-start waits for this reading to drop when the light turns on.
    wait_for_light_sensor = AnalogSensor(port=5)

    # ── Line sensors ───────────────────────────────────────────────────────────
    # Two downward reflectance sensors at the front of the chassis.
    front_left_ir_sensor = IRSensor(port=1)
    front_right_ir_sensor = IRSensor(port=2)

    # A SensorGroup bundles the left/right pair. Missions use it for line work
    # (Defs.front.left, Defs.front.right, follow_line helpers) without naming the
    # individual sensors everywhere.
    front = SensorGroup(
        left=front_left_ir_sensor,
        right=front_right_ir_sensor,
    )

    # ── Drive motors ───────────────────────────────────────────────────────────
    # ticks_to_rad converts encoder ticks → radians of wheel rotation. These are
    # baseline values; the calibration step in the setup mission auto-corrects
    # small deviations at the start of every run, so they do not need to be exact.
    left_motor = Motor(
        port=0,
        inverted=False,
        calibration=MotorCalibration(ticks_to_rad=1.741e-05, vel_lpf_alpha=1.0),
    )
    right_motor = Motor(
        port=1,
        inverted=True,  # right side is physically mirrored → invert direction
        calibration=MotorCalibration(ticks_to_rad=1.712e-05, vel_lpf_alpha=1.0),
    )

    # ── Arm servo ──────────────────────────────────────────────────────────────
    # Each named position becomes a callable step: arm_servo.up(), .hold(), .down()
    # Pass a speed to ease the motion: arm_servo.down(speed=120)  # 120 deg/s
    arm_servo = ServoPreset(
        Servo(port=0),
        positions={
            "up": 20,     # fully raised — safe travel position
            "hold": 90,   # horizontal — holds the object while driving
            "down": 160,  # fully lowered — pick-up / drop-off position
        },
    )

    # ── Claw servo ─────────────────────────────────────────────────────────────
    claw_servo = ServoPreset(
        Servo(port=1),
        positions={
            "open": 40,
            "closed": 150,
        },
    )

    # Sensors that need periodic analog reads (the calibration step samples these).
    analog_sensors = [front_left_ir_sensor, front_right_ir_sensor]


# The generator also exposes a ready-made instance next to the class.
defs = Defs()

__all__ = ["Defs", "defs"]
