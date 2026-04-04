from libstp import (
    DigitalSensor,
    IRSensor,
    Motor,
    MotorCalibration,
    SensorGroup,
    Servo,
    ServoPreset,
)
from libstp import IMU as Imu


class Defs:
    # ── Inertial measurement unit ──────────────────────────────────────────
    imu = Imu()

    # ── Start button (port 10 is the standard Wombat button) ──────────────
    button = DigitalSensor(port=10)

    # ── Line sensors ───────────────────────────────────────────────────────
    # Both sensors are mounted at the front of the chassis.
    front_left_ir_sensor  = IRSensor(port=1)
    front_right_ir_sensor = IRSensor(port=2)

    # SensorGroup lets you call drive_until_black() / strafe helpers on the
    # pair without referring to individual sensors in mission code.
    front = SensorGroup(
        left=front_left_ir_sensor,
        right=front_right_ir_sensor,
    )

    # ── Drive motors ───────────────────────────────────────────────────────
    # ticks_to_rad converts encoder ticks to radians of wheel rotation.
    # Measure this with the Raccoon calibration tool, or use the auto-tune.
    left_motor = Motor(
        port=0,
        inverted=False,
        calibration=MotorCalibration(ticks_to_rad=1.862e-05, vel_lpf_alpha=1.0),
    )
    right_motor = Motor(
        port=1,
        inverted=True,                # right side is physically mirrored
        calibration=MotorCalibration(ticks_to_rad=1.594e-05, vel_lpf_alpha=1.0),
    )

    # ── Arm servo ─────────────────────────────────────────────────────────
    # Named positions become methods: arm_servo.up(), .hold(), .down()
    arm_servo = ServoPreset(
        Servo(port=0),
        positions={
            "up":   20,   # fully raised — safe travel position
            "hold": 90,   # horizontal — holds object while driving
            "down": 160,  # fully lowered — pick-up / drop-off position
        },
    )

    # ── Claw servo ────────────────────────────────────────────────────────
    claw_servo = ServoPreset(
        Servo(port=1),
        positions={
            "open":   40,
            "closed": 150,
        },
    )

    # Sensors that need periodic analog reads (used by the calibration step)
    analog_sensors = [front_left_ir_sensor, front_right_ir_sensor]


__all__ = ["Defs"]
