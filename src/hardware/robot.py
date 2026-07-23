"""Robot definition for ExampleBot.

NORMALLY GENERATED. Like defs.py, `raccoon run` regenerates this from
config/robot.yml + config/missions.yml. It is hand-written here so you can read
the YAML and the resulting Python together.

This is the assembled robot: drivetrain, velocity controller, motion PID,
geometry, and the ordered list of missions. `GenericRobot.start()` (called from
main.py) drives the whole match from these attributes.
"""

from raccoon.drive import (
    AxisVelocityControlConfig,
    ChassisVelocityControlConfig,
    Drive,
)
from raccoon.foundation import Feedforward, PidConfig, PidGains
from raccoon.kinematics_differential import DifferentialKinematics
from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig
from raccoon.robot.api import GenericRobot
from raccoon.robot.geometry import SensorPosition

from src.hardware.defs import Defs
from src.missions.m00_setup_mission import M00SetupMission
from src.missions.m01_navigate_to_object_mission import M01NavigateToObjectMission
from src.missions.m02_collect_object_mission import M02CollectObjectMission
from src.missions.m03_deliver_object_mission import M03DeliverObjectMission
from src.missions.m99_shutdown_mission import M99ShutdownMission


def _build_chassis_vel_config(vx=None, vy=None, wz=None) -> ChassisVelocityControlConfig:
    """Build a velocity control config, leaving unset axes at their defaults."""
    cfg = ChassisVelocityControlConfig()
    if vx is not None:
        cfg.vx = vx
    if vy is not None:
        cfg.vy = vy
    if wz is not None:
        cfg.wz = wz
    return cfg


class Robot(GenericRobot):
    defs = Defs()

    # ── Drivetrain ─────────────────────────────────────────────────────────────
    # DifferentialKinematics for a two-wheeled (tank-drive) robot.
    # wheel_radius and wheelbase are in metres — measure them for your chassis.
    kinematics = DifferentialKinematics(
        left_motor=defs.left_motor,
        right_motor=defs.right_motor,
        wheel_radius=0.0345,  # 34.5 mm rubber wheel
        wheelbase=0.16,       # 160 mm between wheel contact points
    )

    # ── Velocity controller ─────────────────────────────────────────────────────
    # Pure feed-forward (kV=1.0) works for most robots — the STM32 runs the real
    # per-motor PID. Add chassis PID gains only if you see drift at constant speed.
    # `raccoon calibrate` writes accurate kS/PID values here.
    drive = Drive(
        kinematics=kinematics,
        vel_config=_build_chassis_vel_config(
            vx=AxisVelocityControlConfig(
                pid=PidGains(kp=0.0, ki=0.0, kd=0.0),
                ff=Feedforward(kS=0.0, kV=1.0, kA=0.0),
            ),
            wz=AxisVelocityControlConfig(
                pid=PidGains(kp=0.0, ki=0.0, kd=0.0),
                ff=Feedforward(kS=0.0, kV=1.0, kA=0.0),
            ),
        ),
        imu=defs.imu,
    )

    # ── Motion PID ──────────────────────────────────────────────────────────────
    # High-level planner gains for drive_forward / turn_* steps. Tune these if the
    # robot overshoots or oscillates; `raccoon calibrate` sets good starting values.
    motion_pid_config = UnifiedMotionPidConfig(
        distance=PidConfig(
            kp=3.0,
            ki=0.0,
            kd=0.0,
            integral_max=10.0,
            integral_deadband=0.01,
            derivative_lpf_alpha=0.5,
            output_min=-10.0,
            output_max=10.0,
        ),
        heading=PidConfig(
            kp=6.0,
            ki=0.0,
            kd=0.15,
            integral_max=10.0,
            integral_deadband=0.01,
            derivative_lpf_alpha=0.5,
            output_min=-10.0,
            output_max=10.0,
        ),
        velocity_ff=1.0,
        distance_tolerance_m=0.005,  # 5 mm position tolerance
        angle_tolerance_rad=0.017,   # ~1° heading tolerance
        saturation_derating_factor=0.85,
        saturation_min_scale=0.2,
        saturation_recovery_rate=0.02,
        saturation_hold_cycles=5,
        saturation_recovery_threshold=0.95,
        heading_saturation_derating_factor=0.85,
        heading_min_scale=0.25,
        heading_recovery_rate=0.05,
        heading_saturation_error_rad=0.01,
        heading_recovery_error_rad=0.005,
        # Motion constraints are REQUIRED — a zero max_velocity means the robot
        # sits still even when commanded to move.
        linear=AxisConstraints(
            max_velocity=0.24,   # m/s
            acceleration=0.28,   # m/s²
            deceleration=2.05,   # m/s²  (high = sharp, precise stops)
        ),
        lateral=AxisConstraints(
            max_velocity=0.0,    # differential drive cannot strafe
            acceleration=0.0,
            deceleration=0.0,
        ),
        angular=AxisConstraints(
            max_velocity=2.0,    # rad/s
            acceleration=3.0,    # rad/s²
            deceleration=3.0,
        ),
    )

    # ── Robot geometry ───────────────────────────────────────────────────────────
    # Lets the planner account for sensor offsets and project the footprint during
    # wall alignment. forward_cm: +towards front; strafe_cm: +towards left side.
    width_cm = 18.0
    length_cm = 22.0
    rotation_center_forward_cm = -4.0  # rotation pivot sits behind chassis centre
    rotation_center_strafe_cm = 0.0

    _sensor_positions = {
        defs.front_left_ir_sensor: SensorPosition(
            forward_cm=9.0, strafe_cm=6.0, clearance_cm=1.0
        ),
        defs.front_right_ir_sensor: SensorPosition(
            forward_cm=9.0, strafe_cm=-6.0, clearance_cm=1.0
        ),
    }

    # ── Missions ─────────────────────────────────────────────────────────────────
    # setup_mission runs once before the match (calibration, homing).
    # missions run in order; each is advanced to by a button/checkpoint.
    # shutdown_mission runs when `shutdown_in` seconds elapse — always leave the
    # robot in a safe state (servos + motors off) there.
    shutdown_in = 120  # seconds until auto-shutdown (prevents runaway robots)
    setup_mission = M00SetupMission()
    shutdown_mission = M99ShutdownMission()
    missions = [
        M01NavigateToObjectMission(),
        M02CollectObjectMission(),
        M03DeliverObjectMission(),
    ]

    # ── Odometry ─────────────────────────────────────────────────────────────────
    # Odometry is created lazily from the active platform (mock on a laptop,
    # wombat on the Pi). This is exactly what the generator emits — the platform
    # decides how encoders and the IMU are fused, so mission code never has to.
    @property
    def odometry(self):
        if not hasattr(self, "_odometry"):
            from raccoon.hal import platform as _platform

            self._odometry = _platform.Platform.create_odometry(self.kinematics)
        return self._odometry


__all__ = ["Robot"]
