package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Optional;

/**
 * Main competition TeleOp for the robot in preparation for the FTC State Championship in Bozeman.
 * This OpMode uses a "Driver/Operator" control scheme.
 *
 * Features:
 * - Continuous localization and field-aware positioning via Pedro Pathing.
 * - Automated turret aiming that constantly points at the selected goal.
 * - Automated flywheel speed control that adjusts based on the robot's distance to the goal.
 * - Full manual overrides for all automated systems.
 * - Automated diverter gate control with manual override.
 *
 * --- CONTROLLER LAYOUT ---
 *
 * === GAMEPAD 1 (Driver/Operator) ===
 * Left Stick Y:          Drive Forward/Backward
 * Left Stick X:          Strafe Left/Right
 * Right Stick X:         Turn Robot
 * Right Trigger:         Launch from scoop
 * Left Trigger:          Run intake
 * Right Bumper:          Transfer GREEN ball to scoop
 * Left Bumper:           Transfer PURPLE ball to scoop
 * D-Pad Left:            Set pixel diverter to PURPLE path
 * D-Pad Right:           Set pixel diverter to GREEN path
 * D-Pad Up:              Set pixel diverter to NEUTRAL (for clearing jams)
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
@TeleOp(name = "BozemanTeleop")
public class BozemanTeleop extends OpMode {

    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private Follower follower;

    private GameState.Alliance alliance;

    // Button press trackers for edge detection
    private boolean g1_y_pressed, g1_x_pressed, g1_b_pressed, g1_right_bumper_pressed, g1_left_bumper_pressed, g1_a_pressed, g1_start_pressed;
    private boolean was_manually_moving_turret;

    private int green_ball_count = 0;
    private int purple_ball_count = 0;
    private boolean auto_diverter_enabled = true;

    @Override
    public void init() {
        // The RobotHardwareContainer now correctly creates and initializes all hardware,
        // including the mecanum drive.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap, robot);
        robot.initTurret(follower, hardwareMap);
        robot.initLauncher(follower, robot.turret, hardwareMap);
        robot.initDriverAssist(follower);
        robot.initLimelight(hardwareMap, telemetry);

        actionManager = new ActionManager(robot);

        if (GameState.currentPose != null) {
            follower.setStartingPose(GameState.currentPose);
        } else {
            follower.setStartingPose(new Pose());
        }
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        telemetry.addLine("Bozeman TeleOp Initialized. Ready for match!");
        telemetry.addLine("Press PLAY when localization is stable.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        Pose currentPose = follower.getPose();
        telemetry.addLine("Waiting for start...");
        if (currentPose != null) {
            telemetry.addData("X", "%.2f", currentPose.getX());
            telemetry.addData("Y", "%.2f", currentPose.getY());
            telemetry.addData("H", "%.1f", Math.toDegrees(currentPose.getHeading()));
        } else {
            telemetry.addLine("Localization not yet stable...");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        robot.turret.calibrate();
    }

    @Override
    public void loop() {
        // Pedro Pathing continues to handle localization.
        follower.update();

        // The other subsystems continue to update.
        actionManager.update();
        robot.turret.update(alliance);
        robot.launcher.update();

        // Correctly drive the robot using the DriverAssist class.
        robot.driverAssist.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, alliance);

        handleControls();
        updateButtonStates();
        updateTelemetry();
    }

    private void handleControls() {
        // === Gamepad 1: Driver/Operator Controls ===

        // Drive controls are handled by DriverAssist

        // Intake
        if (gamepad1.left_trigger > 0.1) {
            robot.intake.run();
        } else if (gamepad2.a) {
            robot.intake.reverse();
        } else if (!actionManager.isBusy()) {
            robot.intake.stop();
        }

        // Transfer and Launch
        if (gamepad1.right_bumper && !g1_right_bumper_pressed) {
            actionManager.startTransferGreen();
        }
        if (gamepad1.left_bumper && !g1_left_bumper_pressed) {
            actionManager.startTransferPurple();
        }

        if (gamepad1.right_trigger > 0.1) {
            actionManager.launchFromScoop();
        }

        if (gamepad1.a && !g1_a_pressed) robot.launcher.toggleLauncher();

        // Turret
        double turret_input = -gamepad2.right_stick_x; // Fallback manual control on gamepad 2

        boolean is_manually_moving_turret = Math.abs(turret_input) > 0.1;

        if (is_manually_moving_turret) {
            robot.turret.setManualControl(turret_input);
        } else if (was_manually_moving_turret) {
            robot.turret.setModeToAuto();
        }

        if (gamepad1.x && !g1_x_pressed) robot.turret.resetNudge();
        if (gamepad1.y && !g1_y_pressed) robot.turret.toggleAutoAim();
        if (gamepad1.start && !g1_start_pressed) robot.driverAssist.resetHeading();

        // Diverter
        if (gamepad1.dpad_left) {
            auto_diverter_enabled = false;
            actionManager.setDiverterToPurple();
        } else if (gamepad1.dpad_right) {
            auto_diverter_enabled = false;
            actionManager.setDiverterToGreen();
        } else if (gamepad1.dpad_up) {
            auto_diverter_enabled = false;
            actionManager.setDiverterToNeutral();
        } else if (gamepad1.dpad_down) { // Example: use dpad down to re-enable auto diverter
            auto_diverter_enabled = true;
        }

        if(auto_diverter_enabled) {
            handleAutoDiverter();
        }

        // Drive Mode
        if (gamepad1.b && !g1_b_pressed) {
            switch (robot.driverAssist.getMode()) {
                case ROBOT_CENTRIC: robot.driverAssist.setMode(DriverAssist.DriveMode.FIELD_CENTRIC); break;
                case FIELD_CENTRIC: robot.driverAssist.setMode(DriverAssist.DriveMode.TARGET_LOCK); break;
                case TARGET_LOCK: robot.driverAssist.setMode(DriverAssist.DriveMode.ROBOT_CENTRIC); break;
            }
        }
    }

    private void handleAutoDiverter() {
        // Set pipeline to detect green balls
        robot.limelightBallDetector.setPipeline(1);
        Optional<LLResult.Target> greenBall = robot.limelightBallDetector.getLargestBall();

        // Set pipeline to detect purple balls
        robot.limelightBallDetector.setPipeline(2);
        Optional<LLResult.Target> purpleBall = robot.limelightBallDetector.getLargestBall();

        DiverterHardware.GatePosition targetPosition = DiverterHardware.GatePosition.NEUTRAL;

        if (greenBall.isPresent() && purpleBall.isPresent()) {
            if (greenBall.get().getArea() > purpleBall.get().getArea()) {
                targetPosition = DiverterHardware.GatePosition.GREEN;
            } else {
                targetPosition = DiverterHardware.GatePosition.PURPLE;
            }
        } else if (greenBall.isPresent()) {
            targetPosition = DiverterHardware.GatePosition.GREEN;
        } else if (purpleBall.isPresent()) {
            targetPosition = DiverterHardware.GatePosition.PURPLE;
        }

        // Check if the target track is full
        if (targetPosition == DiverterHardware.GatePosition.GREEN && green_ball_count >= 2) {
            targetPosition = DiverterHardware.GatePosition.NEUTRAL;
        } else if (targetPosition == DiverterHardware.GatePosition.PURPLE && purple_ball_count >= 2) {
            targetPosition = DiverterHardware.GatePosition.NEUTRAL;
        }

        robot.diverter.setPosition(targetPosition);

        // Check for jams and increment ball counts
        if (robot.diverter.isStuck()) {
            // If stuck, assume the ball is blocking the gate and increment the count for the other side
            if (targetPosition == DiverterHardware.GatePosition.GREEN) {
                purple_ball_count++;
            } else if (targetPosition == DiverterHardware.GatePosition.PURPLE) {
                green_ball_count++;
            }
            robot.diverter.setPosition(DiverterHardware.GatePosition.NEUTRAL);
        } else {
            // If not stuck, assume the ball was sorted correctly
            if (targetPosition == DiverterHardware.GatePosition.GREEN) {
                green_ball_count++;
            } else if (targetPosition == DiverterHardware.GatePosition.PURPLE) {
                purple_ball_count++;
            }
        }
    }

    private void updateButtonStates() {
        // Gamepad 1
        g1_y_pressed = gamepad1.y;
        g1_x_pressed = gamepad1.x;
        g1_a_pressed = gamepad1.a;
        g1_start_pressed = gamepad1.start;
        g1_right_bumper_pressed = gamepad1.right_bumper;
        g1_left_bumper_pressed = gamepad1.left_bumper;
        g1_b_pressed = gamepad1.b;

        double turret_input = -gamepad2.right_stick_x;
        was_manually_moving_turret = Math.abs(turret_input) > 0.1;
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Drive Mode", robot.driverAssist.getMode().toString());
        telemetry.addData("Turret Auto-Aim", robot.turret.isAutoAimActive ? "ACTIVE" : "OFF");
        telemetry.addData("Launcher Target RPM", "%.0f", robot.launcher.getCurrentTargetVelocity());
        telemetry.addData("Launcher State", robot.launcher.isLauncherOn() ? "ON" : "OFF");
        telemetry.addData("Turret Current (A)", "%.2f", robot.turret.getTurretCurrent());
        telemetry.addData("Intake Current (A)", "%.2f", robot.intake.getIntakeCurrent());
        telemetry.addData("Auto Diverter", auto_diverter_enabled ? "ON" : "OFF");
        telemetry.addData("Green Balls", green_ball_count);
        telemetry.addData("Purple Balls", purple_ball_count);
        telemetry.addData("Diverter is Stuck", robot.diverter.isStuck());

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
