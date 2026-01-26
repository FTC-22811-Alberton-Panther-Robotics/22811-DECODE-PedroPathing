package org.firstinspires.ftc.teamcode.Competition;

import static java.lang.Double.NaN;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * ---------------------------------------------------------------------------------
 * --- MASTER CHECKLIST AND CHANGELOG ---
 * ---------------------------------------------------------------------------------
 * This is the central guide for ensuring the robot is competition-ready.
 *
 * -- V1.1 CHANGELOG --
 * - MAJOR BUGFIX: Corrected a critical bug where the dead-wheel localizer was using
 *   zeroes for its pod offsets, causing significant localization drift. The correct
 *   offsets are now in `CustomPinpointConstants.java`.
 * - Refactored all subsystems to use the correct game piece term: "Artifact".
 * - Implemented a Region of Interest (ROI) in the Limelight detector to prevent it
 *   from seeing Artifacts already inside the robot.
 * - Created detailed Javadoc and tuning guides in all major hardware classes.
 * - Decoupled the transfer and launch sequences for more precise driver control.
 * - Consolidated primary controls to Gamepad 1 for a single operator.
 * - Confirmed reliable fallback controls exist for all automated systems.
 * - Added `isPoseReliable` checks to prevent auto-aim and auto-velocity from using bad data.
 *
 * -- MASTER TUNING AND TESTING CHECKLIST --
 * DRIVETRAIN & LOCALIZATION:
 * [ ] (CRITICAL) DRIVE TUNING: Use `DriveTuning` OpMode to find `X_VELOCITY`, `Y_VELOCITY`,
 *     and `staticFrictionCoefficient` in `CustomMecanumDrive`.
 * [x] (CRITICAL) ODOMETRY OFFSETS: Verify dead-wheel pod offsets (`forwardPodY`, `strafePodX`)
 *     in `CustomPinpointConstants` are accurately measured. (Values updated from bugfix).
 * [ ] MOTOR DIRECTIONS: Verify directions for both the main drive motors (`CustomMecanumDrive`)
 *     and the dead-wheel encoders (`CustomPinpointConstants`).
 * [x] IMU ORIENTATION: Verify the Control Hub orientation in `MecanumHardware` for field-relative fallbacks.
 *
 * TURRET:
 * [ ] (CRITICAL) PIDF TUNING: Run the `TurretTuningOpMode` to tune the `TURRET_PIDF`
 *     coefficients for fast, stable aiming.
 * [ ] PHYSICAL CONSTANTS: Verify `GEAR_RATIO`, `MOTOR_TICKS_PER_REV`, `ZERO_POINT_DEGREES`,
 *     and software limits in `TurretHardware`.
 * [ ] CALIBRATION: Tune `CALIBRATION_POWER` and `CALIBRATION_TIME_MS` for a gentle but
 *     reliable calibration sequence.
 *
 * LAUNCHER:
 * [ ] VELOCITY MAPPING: Tune the min/max shot distances and velocities in `LauncherHardware`.
 * [ ] MOTOR CONSTANTS: Verify motor directions and `ticksPerRevolution` in `LauncherHardware`.
 *
 * DIVERTER & INTAKE:
 * [ ] VOLTAGE CALIBRATION: Calibrate analog feedback voltages in `DiverterHardware`.
 * [ ] LIMELIGHT ROI: Tune `ROI_MIN_Y` and `ROI_MAX_Y` in `DiverterHardware`.
 * [ ] JAM DETECTION: Tune `STUCK_TOLERANCE` in `DiverterHardware`.
 * [ ] LOGIC VALIDATION: Physically test the automatic artifact counting logic.
 * [ ] INTAKE POWER: Tune `INTAKE_POWER` in `IntakeHardware`.
 *
 * SEQUENCES (`ActionManager`):
 * [ ] TIMING: Tune all `timer.seconds()` durations for transfer and launch sequences.
 *
 * GAME & FIELD SPECIFIC:
 * [ ] FIELD POSES: Verify all autonomous poses in `FieldPosePresets` on a physical field.
 * [ ] APRILTAG IDS: Verify the AprilTag IDs in `GameState` match the official game manual.
 * [ ] LIMELIGHT PIPELINES: Ensure Pipelines 0 (AprilTag), 1 (Green), and 2 (Purple) are
 *     correctly configured in the Limelight web interface.
 * ---------------------------------------------------------------------------------
 *
 * --- CONTROLLER LAYOUT ---
 *
 * === GAMEPAD 1 (Driver/Operator) ===
 * Left Stick Y:          Drive Forward/Backward
 * Left Stick X:          Strafe Left/Right
 * Right Stick X:         Turn Robot
 * Right Trigger:         Launch Staged Artifact from scoop
 * Left Trigger:          Run intake
 * Right Bumper:          Transfer GREEN Artifact to scoop
 * Left Bumper:           Transfer PURPLE Artifact to scoop
 * D-Pad Left/Right:      Manually set diverter to PURPLE/GREEN path
 * D-Pad Up:              Manually set diverter to NEUTRAL
 * D-Pad Down:            Re-enable auto-diverter mode
 * A Button:              Toggle flywheel motors ON/OFF
 * Y Button:              Toggle turret auto-aim system ON/OFF
 * X Button:              Reset any turret nudge adjustment
 * B Button:              Toggle drive mode (Robot-Centric, Field-Centric, Target-Lock)
 * Start Button:          Reset robot heading to 90 degrees
 *
 * === GAMEPAD 2 (Fallback) ===
 * Right Stick X-Axis:    Manual turret rotation override
 * A Button:              Run intake in reverse
 */
@TeleOp(name = "BozemanTeleop", group = "01 Bozeman")
public class BozemanTeleop extends OpMode {

    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private Follower follower;
    private GameState.Alliance alliance;
    private enum StartPosition {FRONT, BACK};
    private StartPosition startPosition = StartPosition.FRONT;

    // Drive Mode Logic
    public enum DriveMode { ROBOT_CENTRIC, FIELD_CENTRIC, TARGET_LOCK }
    private DriveMode currentDriveMode = DriveMode.ROBOT_CENTRIC;
    private static final double HEADING_KP = 0.8; // Proportional gain for Target Lock

    // State variables for edge detection and controls
    private boolean was_manually_moving_turret;
    private enum StagedArtifact { NONE, GREEN, PURPLE }
    private StagedArtifact stagedArtifact = StagedArtifact.NONE;

    private boolean auto_diverter_enabled = true;

    @Override
    public void init() {
        // Create the hardware container
        robot = new RobotHardwareContainer(hardwareMap, telemetry);

        // Create and get the single, authoritative instances of the Follower and Localizer
        follower = Constants.createFollower(hardwareMap, telemetry);

        // Initialize hardware that depends on the follower
        robot.initTurret(follower, hardwareMap);
        robot.initLauncher(follower, robot.turret, hardwareMap);

        actionManager = new ActionManager(robot);

        // Default to Blue Alliance, but allow selection during init.
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        telemetry.addLine("Bozeman TeleOp Initialized. Ready for match!");
        telemetry.addLine("INIT: D-Pad L/R for Alliance, Up/Down for Start Pos");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Allow alliance selection during init
        if (gamepad1.dpad_left) alliance = GameState.Alliance.BLUE;
        if (gamepad1.dpad_right) alliance = GameState.Alliance.RED;
        if (gamepad1.dpad_up) startPosition = StartPosition.BACK;
        if (gamepad1.dpad_down) startPosition = StartPosition.FRONT;

        GameState.alliance = this.alliance;
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Start", startPosition);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.turret.calibrate();
        // If we have a pose from Autonomous, set it. Otherwise, default to a known start for testing.

        if (GameState.currentPose != null) {
            follower.setStartingPose(GameState.currentPose);
        } else if (alliance == GameState.Alliance.RED) {
            follower.setStartingPose((startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START);
        } else {
            // Default to Blue Front Start if no auto pose exists. This provides a known
            // starting point for testing field-relative features.
            follower.setStartingPose((startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START);
        }
    }

    @Override
    public void loop() {
        // Update all subsystems on every loop.
        follower.update();
        actionManager.update();
        robot.turret.update(alliance);
        robot.launcher.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        // This block now contains all the drive logic, calling the follower directly.
//        switch (currentDriveMode) {
//            case ROBOT_CENTRIC:
//                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//                break;
//            case FIELD_CENTRIC:
//                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
//                break;
//            case TARGET_LOCK:
//                Pose robotPose = Constants.localizer.getPose();
//                if (robotPose != null) {
//                    double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(robotPose), robotPose.getHeading());
//                    double calculatedTurn = HEADING_KP * headingError;
//                    follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, calculatedTurn, false);
//                }
//                break;
//        }

        if (auto_diverter_enabled) {
            robot.diverter.update();
        }

        handleControls();
        
        was_manually_moving_turret = Math.abs(-gamepad2.right_stick_x) > 0.1;
        
        updateTelemetry();
    }

    private void handleControls() {
        // Intake
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.run();
        } else if (gamepad2.a) {
            robot.intake.reverse();
        } else if (!actionManager.isBusy()) {
            robot.intake.stop();
        }

        // Transfer and Staging
        if (gamepad1.rightBumperWasPressed()) {
            actionManager.startTransferGreenArtifact();
            stagedArtifact = StagedArtifact.GREEN;
        }
        if (gamepad1.leftBumperWasPressed()) {
            actionManager.startTransferPurpleArtifact();
            stagedArtifact = StagedArtifact.PURPLE;
        }

        // Launching
        if (gamepad1.right_trigger > 0.1) {
            actionManager.launchFromScoop();
            if (stagedArtifact == StagedArtifact.GREEN) {
                robot.diverter.decrementGreenArtifactCount();
            } else if (stagedArtifact == StagedArtifact.PURPLE) {
                robot.diverter.decrementPurpleArtifactCount();
            }
            stagedArtifact = StagedArtifact.NONE;
        }

        if (gamepad1.aWasPressed()) robot.launcher.toggleLauncher();

        // Turret
        if (robot.turret.isCalibrated()) {
            double turret_input = -gamepad2.right_stick_x;
            if (Math.abs(turret_input) > 0.1) {
                robot.turret.setManualControl(turret_input);
            } else if (was_manually_moving_turret) {
                robot.turret.setModeToAuto();
            }
            if (gamepad1.xWasPressed()) robot.turret.resetNudge();
            if (gamepad1.yWasPressed()) robot.turret.toggleAutoAim();
        }

        /**
          * Reset heading with start button, or set entire pose to (0, 0, 0) if it isn't set yet.
          */
        if (gamepad1.startWasPressed()) {
            if (follower.getPose() == null || Double.isNaN(follower.getPose().getX())){
                follower.setPose(new Pose(0, 0, 0));
            } else {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(90)));
            }
        }

        // Diverter Manual Override
        if (gamepad1.dpad_left) {
            auto_diverter_enabled = false;
            robot.diverter.setPosition(DiverterHardware.GatePosition.PURPLE);
        } else if (gamepad1.dpad_right) {
            auto_diverter_enabled = false;
            robot.diverter.setPosition(DiverterHardware.GatePosition.GREEN);
        } else if (gamepad1.dpad_up) {
            auto_diverter_enabled = false;
            robot.diverter.setPosition(DiverterHardware.GatePosition.NEUTRAL);
        } else if (gamepad1.dpadDownWasPressed()) {
            auto_diverter_enabled = true;
        }

        // Drive Mode
        if (gamepad1.bWasPressed()) {
            switch (currentDriveMode) {
                case ROBOT_CENTRIC: currentDriveMode = DriveMode.FIELD_CENTRIC; break;
                case FIELD_CENTRIC: currentDriveMode = DriveMode.TARGET_LOCK; break;
                case TARGET_LOCK: currentDriveMode = DriveMode.ROBOT_CENTRIC; break;
            }
        }

        // --- Alliance Toggle for Testing ---
        if (gamepad2.dpadRightWasPressed()) {
            alliance = GameState.Alliance.RED;
            GameState.alliance = alliance;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            alliance = GameState.Alliance.BLUE;
            GameState.alliance = alliance;
        }
    }
    public double calculateHeadingToGoal(Pose robotPose) {
        Pose targetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;
        return Math.atan2(targetGoal.getY() - robotPose.getY(), targetGoal.getX() - robotPose.getX());
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Drive Mode", currentDriveMode.toString());
        telemetry.addData("Turret Calibrated", robot.turret.isCalibrated());
        telemetry.addData("Turret Auto-Aim", robot.turret.isAutoAimActive ? "ACTIVE" : "OFF");
        telemetry.addData("Launcher State", robot.launcher.isLauncherOn() ? "ON" : "OFF");
        telemetry.addData("Auto Diverter", auto_diverter_enabled ? "ON" : "OFF");
        telemetry.addData("Staged Artifact", stagedArtifact.toString());
        telemetry.addData("Green Artifacts", robot.diverter.getGreenArtifactCount());
        telemetry.addData("Purple Artifacts", robot.diverter.getPurpleArtifactCount());

        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            telemetry.addData("Robot X", "%.2f", currentPose.getX());
            telemetry.addData("Robot Y", "%.2f", currentPose.getY());
            telemetry.addData("Robot H", "%.1f", Math.toDegrees(currentPose.getHeading()));
        } else {
            telemetry.addLine("Pose: Initializing...");
        }
        telemetry.update();
    }
}
