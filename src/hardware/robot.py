from libstp import (
    AxisConstraints,
    AxisVelocityControlConfig,
    ChassisVelocityControlConfig,
    DifferentialKinematics,
    Drive,
    Feedforward,
    FusedOdometry,
    FusedOdometryConfig,
    GenericRobot,
    PidConfig,
    PidGains,
    SensorPosition,
    UnifiedMotionPidConfig,
)

from src.hardware.defs import Defs
from src.missions.m00_setup_mission import M00SetupMission
from src.missions.m01_navigate_to_object_mission import M01NavigateToObjectMission
from src.missions.m02_collect_object_mission import M02CollectObjectMission
from src.missions.m03_deliver_object_mission import M03DeliverObjectMission


def _vel_config(vx=None, vy=None, wz=None) -> ChassisVelocityControlConfig:
    """Build a velocity control config, keeping unset axes at their defaults."""
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

    # ── Drivetrain ────────────────────────────────────────────────────────
    # DifferentialKinematics for a two-wheeled (tank-drive) robot.
    # wheel_radius and wheelbase must be measured in metres.
    kinematics = DifferentialKinematics(
        left_motor=defs.left_motor,
        right_motor=defs.right_motor,
        wheel_radius=0.0345,   # 34.5 mm rubber wheel
        wheelbase=0.16,        # 160 mm between wheel contact points
    )

    # ── Velocity controller ───────────────────────────────────────────────
    # Pure feedforward (kV=1.0) works well for most robots without slip.
    # Add PID gains (kp, ki, kd) if wheel slip or surface irregularities
    # cause drift at constant speed.
    drive = Drive(
        kinematics=kinematics,
        vel_config=_vel_config(
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

    # ── Odometry ──────────────────────────────────────────────────────────
    # FusedOdometry blends wheel encoders and IMU for accurate position.
    # bemf_trust=1.0 weights the encoder estimate fully; lower values blend
    # in IMU when encoders are unreliable (e.g. on slippery surfaces).
    odometry = FusedOdometry(
        imu=defs.imu,
        kinematics=kinematics,
        config=FusedOdometryConfig(bemf_trust=1.0),
    )

    # ── Motion PID ────────────────────────────────────────────────────────
    # These gains control the high-level motion planner (drive_forward,
    # turn_right, etc.). Tune with the Raccoon auto-tune tool, or start
    # with the values below and adjust empirically.
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
        distance_tolerance_m=0.005,    # 5 mm position tolerance
        angle_tolerance_rad=0.017,     # ~1 ° heading tolerance
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
        linear=AxisConstraints(
            max_velocity=0.24,    # m/s
            acceleration=0.28,    # m/s²
            deceleration=2.05,    # m/s²  (high = sharp stops)
        ),
        lateral=AxisConstraints(
            max_velocity=0.0,     # not used for differential drive
            acceleration=0.0,
            deceleration=0.0,
        ),
        angular=AxisConstraints(
            max_velocity=2.0,     # rad/s
            acceleration=3.0,     # rad/s²
            deceleration=3.0,
        ),
    )

    # ── Robot geometry ────────────────────────────────────────────────────
    # Used by the motion planner to account for sensor offsets and to
    # correctly project the robot's footprint during wall alignment.
    width_cm  = 18.0
    length_cm = 22.0
    rotation_center_forward_cm = -4.0   # rotation pivot behind robot centre
    rotation_center_strafe_cm  =  0.0

    # Sensor positions relative to the robot's rotation centre.
    # forward_cm: positive = towards front; strafe_cm: positive = left side.
    _sensor_positions = {
        defs.front_left_ir_sensor: SensorPosition(
            forward_cm=9.0, strafe_cm=6.0, clearance_cm=1.0
        ),
        defs.front_right_ir_sensor: SensorPosition(
            forward_cm=9.0, strafe_cm=-6.0, clearance_cm=1.0
        ),
    }

    # ── Missions ──────────────────────────────────────────────────────────
    # setup_mission runs once before the match starts (calibration, etc.).
    # missions run in order, each triggered by a button press.
    # shutdown_mission runs automatically when shutdown_in seconds elapse.
    shutdown_in      = 120   # seconds until auto-shutdown
    setup_mission    = M00SetupMission()
    shutdown_mission = None
    missions = [
        M01NavigateToObjectMission(),
        M02CollectObjectMission(),
        M03DeliverObjectMission(),
    ]


__all__ = ["Robot"]
