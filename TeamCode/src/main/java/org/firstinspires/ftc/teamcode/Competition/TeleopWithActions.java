package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Main TeleOp for competition, featuring advanced driver-assistance controls.
 *
 * This OpMode uses a helper class, {@link DriverAssist}, to manage different
 * driving modes, making the robot easier to control during a match. It also uses
 * the persistent {@link GameState} class to seamlessly transition from Autonomous to TeleOp,
 * inheriting the correct alliance color and field position.
 *
 * --- Controller Map ---
 * [GAMEPAD 1]
 * Left Stick Y:          Drive Forward/Backward
 * Left Stick X:          Strafe Left/Right
 * Right Stick X:         Turn Robot
 *
 * D-Pad Up:              Activate TARGET_LOCK Drive Mode
 * D-Pad Left:            Activate FIELD_CENTRIC Drive Mode
 * D-Pad Down:            Activate ROBOT_CENTRIC Drive Mode (Default)
 *
 * Right Trigger (Hold):  Launch Artifacts (Purple Track)
 * Right Bumper (Press):  Start Timed Intake Sequence
 * Left Bumper (Hold):    Reverse All Mechanisms (for clearing jams)
 *
 * Button B (Press):      Trigger AUTO_PARK sequence
 * Button X (Press):      Manually override Alliance selection (in case of error)
 * Button Y (Press):      Manual override for Transfer belt
 *
 * --- Drive Modes Explained ---
 * - ROBOT_CENTRIC: This is the classic, default mode. "Forward" on the joystick is always
 *   the direction the robot's front is facing. This is intuitive for direct maneuvering
 *   but can be confusing when the driver loses track of the robot's orientation.
 *
 * - FIELD_CENTRIC: In this mode, the controls are relative to the field itself. Pushing
 *   "forward" on the joystick will always move the robot down the field (e.g., towards the
 *   opposite wall), regardless of which way the robot is pointing. This is extremely
 *   useful for navigating the field quickly without having to mentally rotate the controls.
 *
 * - TARGET_LOCK: This is the most advanced mode. It combines the field-centric driving
 *   for forward/strafe movement with an automatic heading controller. The robot will
 *   constantly try to point itself at the center of the opponent's goal, freeing the
 *   driver from having to manually aim. This is ideal for quickly lining up shots from
 *   anywhere on the field.
 *
 * --- How it Works ---
 * The {@link DriverAssist} class takes the raw joystick inputs and the robot's current
 * position (from the Follower). Based on the selected drive mode, it performs the necessary
 * math (e.g., rotating vectors for field-centric drive, running a P-controller for target-lock).
 * It then passes the final, calculated drive powers (forward, strafe, turn) to the
 * PedroPathing {@link Follower} using the `setTeleOpDrive()` method. This ensures that all
 * driving, whether manual or autonomous, is routed through the same system for smooth control.
 */
@TeleOp(name = "TeleopWithActions", group = "01 Helena")
public class TeleopWithActions extends OpMode {

    // Hardware and helper class instances
    RobotHardwareContainer robot;
    ActionManager actionManager;
    Follower follower;
    DriverAssist driveHelper;

    // State machine for managing high-level OpMode logic (e.g., switching to auto-park)
    private enum TeleOpState { MANUAL, AUTO_PARK }
    private TeleOpState currentState = TeleOpState.MANUAL;

    // Stores the current alliance, loaded from the persistent GameState class.
    private GameState.Alliance alliance;

    // Button press state trackers to prevent an action from firing multiple times on a single press.
    private boolean dpad_up_pressed, dpad_down_pressed, dpad_left_pressed, dpad_right_pressed;
    private boolean y_pressed, b_pressed, x_pressed, right_bumper_pressed;

    @Override
    public void init() {
        // Initialize all our hardware and helper classes.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap, robot.localizer);
        driveHelper = new DriverAssist(follower);

        // --- Load State from Autonomous ---
        // This is the seamless handoff from Auto to TeleOp.
        // If the GameState class has a pose from the end of auto, set our starting position.
        if (GameState.currentPose != null) {
            follower.setPose(GameState.currentPose);
        }
        // Load the alliance color that was selected during the autonomous init phase.
        // Default to BLUE if for some reason autonomous was never run (e.g., during testing).
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        // Provide clear instructions to the driver on the Driver Station.
        telemetry.addLine("Actions TeleOp Initialized. Ready for match!");
        telemetry.addData("Loaded Alliance", this.alliance);
        telemetry.addLine("DPAD U/L/D -> TargetLock/Field/Robot Centric");
        telemetry.addLine("X -> Override Alliance | B -> Auto-Park");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ALWAYS update the follower and action manager in the main loop.
        // follower.update() runs our fused localization (Pinpoint + Limelight).
        // actionManager.update() runs the state machines for timed actions like intake/launch.
        follower.update();
        actionManager.update();

        // This is the main state machine for TeleOp.
        switch (currentState) {
            case MANUAL:
                // --- Drive Mode Selection ---
                // The D-Pad is used to switch between the three drive modes.
                if (gamepad1.dpad_down && !dpad_down_pressed) driveHelper.setMode(DriverAssist.DriveMode.ROBOT_CENTRIC);
                if (gamepad1.dpad_left && !dpad_left_pressed) driveHelper.setMode(DriverAssist.DriveMode.FIELD_CENTRIC);
                if (gamepad1.dpad_up && !dpad_up_pressed) driveHelper.setMode(DriverAssist.DriveMode.TARGET_LOCK);

                // --- Alliance Override ---
                // In case the robot was restarted or the auto selection was wrong,
                // this allows the driver to manually flip the alliance color.
                if (gamepad1.x && !x_pressed) {
                    alliance = (alliance == GameState.Alliance.BLUE) ? GameState.Alliance.RED : GameState.Alliance.BLUE;
                }

                // --- Drive Logic ---
                // We pass the joystick inputs and alliance to the drive helper.
                // The helper class handles all the complex math and sends the final drive
                // powers to the follower.
                driveHelper.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, alliance);

                // --- Auto-Park Trigger ---
                // The 'B' button triggers a switch to the AUTO_PARK state.
                if (gamepad1.b && !b_pressed) {
                    // Get the correct parking pose based on the current alliance.
                    Pose parkPose = (alliance == GameState.Alliance.BLUE) ? FieldPosePresets.BLUE_BASE : FieldPosePresets.RED_BASE;
                    // Build a simple path from where we are right now to the park pose.
                    Path parkingPath = new Path(new BezierLine(follower.getPose(), parkPose));
                    // Command the follower to execute the path.
                    follower.followPath(parkingPath);
                    // Switch the state machine to AUTO_PARK mode.
                    currentState = TeleOpState.AUTO_PARK;
                }

                // Handle all the other non-driving mechanism controls.
                handleManualMechanisms();
                break;

            case AUTO_PARK:
                // In this state, the robot is autonomously driving.
                telemetry.addLine("--- AUTO-PARKING --- ");
                // If the path is finished OR the driver touches the joysticks, interrupt and return to manual control.
                if (!follower.isBusy() || Math.abs(gamepad1.left_stick_y) > 0.1) {
                    follower.breakFollowing();
                    currentState = TeleOpState.MANUAL;
                }
                break;
        }

        // Update the state of all buttons to detect single presses.
        updateButtonStates();
        // Update the telemetry on the Driver Station screen.
        updateTelemetry();
    }

    /**
     * This method contains the logic for all non-driving manual controls.
     */
    private void handleManualMechanisms() {
        // The new, more intuitive control scheme.
        // LAUNCH (Purple) is on the right trigger, like a video game.
        if (gamepad1.right_trigger > 0.1) {
            actionManager.startLaunch();
        } 
        // INTAKE is on the right bumper. This is a "press" to start a timed sequence.
        else if (gamepad1.right_bumper && !right_bumper_pressed) {
            actionManager.startIntake();
        } 
        // REVERSE ALL is on the left bumper to clear jams.
        else if (gamepad1.left_bumper) {
            actionManager.reverseAll();
        }
        // Manual override for the transfer belt is on 'Y'.
        else if (gamepad1.y && !y_pressed) {
            robot.transfer.run();
        } 
        // If no other actions are being commanded and the ActionManager is not busy, stop all motors.
        // This is a safety feature to prevent mechanisms from running indefinitely.
        else if (!actionManager.isBusy()) {
            actionManager.stopAll();
        }
    }

    /**
     * Updates the state of all gamepad buttons to enable single-press detection (rising edge).
     */
    private void updateButtonStates() {
        dpad_up_pressed = gamepad1.dpad_up;
        dpad_down_pressed = gamepad1.dpad_down;
        dpad_left_pressed = gamepad1.dpad_left;
        dpad_right_pressed = gamepad1.dpad_right;
        right_bumper_pressed = gamepad1.right_bumper;
        y_pressed = gamepad1.y;
        b_pressed = gamepad1.b;
        x_pressed = gamepad1.x;
    }

    /**
     * Updates the telemetry on the driver station with useful information.
     */
    private void updateTelemetry() {
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driveHelper.getMode());
        telemetry.addData("Alliance", alliance.toString());
        // Display the robot's current field-relative position from the fused localizer.
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("H", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything is stopped when the OpMode is ended.
        actionManager.stopAll();
        follower.breakFollowing();
    }
}
