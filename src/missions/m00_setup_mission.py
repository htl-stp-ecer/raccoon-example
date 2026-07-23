from raccoon import *

from src.hardware.defs import Defs
from src.mission_params import MissionParams


class M00SetupMission(SetupMission):
    """Runs ONCE before the match — this is where you prepare, not perform.

    Good things to do in setup:
      - Home every servo to a known position.
      - Ask for any table-side parameters (MissionParams).
      - Calibrate the line sensors and drive distance.
      - Wait for the light-start (or a button in dev mode).

    The setup timer is the tournament's pre-match window. We pause it while the
    human is still positioning the robot and resume it once calibration begins,
    so slow human steps don't eat into the budget.
    """

    # How long the pre-match setup window is, in seconds.
    setup_time = 90

    def sequence(self) -> Sequential:
        return seq([
            # Freeze the setup countdown while the operator gets set up.
            pause_setup_timer(),

            # Home the mechanism to a known, safe state.
            Defs.arm_servo.up(),
            Defs.claw_servo.open(),

            # Ask for the table-side parameter. `.ask()` shows a number pad on the
            # robot screen; the value is remembered (persist=True) for next time.
            MissionParams.approach_distance.ask("Approach distance to object"),

            # Everything below is calibration — skipped by `raccoon run --config dev-nc`
            # (i.e. when no_calibrate is set) so you can iterate without re-running it.
            run_unless_no_calibrate(
                seq([
                    wait_for_button("Place robot on the calibration line, then press"),

                    # Resume the countdown: real setup work starts now.
                    start_setup_timer(),

                    # Lock the current IMU heading as 0° so the calibration drive
                    # is straight and every later turn_to_heading is relative to it.
                    mark_heading_reference(),

                    # Drive forward while sampling the two front IR sensors. The
                    # collected black/white readings become the "default" IR set,
                    # and the drive distance calibrates the FORWARD axis.
                    collect_drive(
                        collect_ir_set(
                            drive_forward(cm=50),
                            set_name="default",
                            sensors=[
                                Defs.front_left_ir_sensor,
                                Defs.front_right_ir_sensor,
                            ],
                        )
                    ),

                    # Refuse to continue unless calibration actually produced the
                    # data we need. Fails loudly here instead of misbehaving mid-match.
                    calibration_gate(
                        require_axes=[CalibrationAxis.FORWARD],
                        require_ir_sets=["default"],
                    ),
                ]),
            ),

            # After setup returns, the runtime automatically waits for the start
            # signal: the light-start sensor in a real match, or the button in
            # dev mode / when no light sensor is configured.
        ])
