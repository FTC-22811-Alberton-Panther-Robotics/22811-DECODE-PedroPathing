package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Main competition TeleOp for the robot Helena.
 * This OpMode uses a "Driver/Operator" control scheme.
 *
 * Features:
 * - Continuous localization and field-aware positioning via Pedro Pathing.
 * - Automated turret aiming that constantly points at the selected goal.
 * - Automated flywheel speed control that adjusts based on the robot's distance to the goal.
 * - Full manual overrides for all automated systems.
 *
 * --- CONTROLLER LAYOUT ---
 *
 * === GAMEPAD 1 (Operator) ===
 * Right/Left Triggers:   Launch Green/Purple pixel (by feeding into flywheels)
 * Right/Left Bumpers:    Fine-tune flywheel speed UP/DOWN
 * Right Stick X-Axis:    Manual turret rotation override
 * D-Pad Right/Left:      Manual turret rotation override
 * A Button:              Toggle flywheel motors ON/OFF
 * Y Button:              Toggle turret auto-aim system ON/OFF
 * X Button:              Reset any turret nudge adjustment
 * Start Button:          Reset robot heading to 90 degrees
 *
 * === GAMEPAD 2 (Driver) ===
 * Left Stick Y:          Drive Forward/Backward
 * Left Stick X:          Strafe Left/Right
 * Right Stick X:         Turn Robot
 * Right/Left Triggers:   Run intake IN/OUT
 * D-Pad Left:            Set pixel diverter to PURPLE path
 * D-Pad Right:           Set pixel diverter to GREEN path
 * D-Pad Up:              Set pixel diverter to NEUTRAL (for clearing jams)
 * B Button:              Toggle drive mode (Robot-Centric, Field-Centric, Target-Lock)
 */
@TeleOp(name = "HelenaTeleOp_Test")
public class HelenaTeleOp_Test extends OpMode {

    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private Follower follower;

    private GameState.Alliance alliance;

    // Button press trackers for edge detection
    private boolean g1_y_pressed, g1_x_pressed, g1_right_bumper_pressed, g1_left_bumper_pressed, g1_a_pressed, g1_start_pressed;
    private boolean g2_b_pressed;
    private boolean was_manually_moving_turret;


    @Override
    public void init() {
        // The RobotHardwareContainer now correctly creates and initializes all hardware,
        // including the mecanum drive.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap, robot);
        robot.initTurret(follower, hardwareMap); 
        robot.initLauncher(follower, robot.turret, hardwareMap);
        robot.initDriverAssist(follower);
        
        actionManager = new ActionManager(robot);

        if (GameState.currentPose != null) {
            follower.setStartingPose(GameState.currentPose);
        } else {
            follower.setStartingPose(new Pose());
        }
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        telemetry.addLine("Helena TeleOp Initialized. Ready for match!");
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
        robot.driverAssist.update(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, alliance);

        handleManualControls();
        updateButtonStates();
        updateTelemetry();
    }

    private void handleManualControls() {
        // === Gamepad 1: Operator Controls (Turret, Shooter) ===
        if (gamepad1.right_trigger > 0.1) actionManager.startGreenBallShoot();
        else if (gamepad1.left_trigger > 0.1) actionManager.startPurpleBallShoot();

        if (gamepad1.right_bumper && !g1_right_bumper_pressed) robot.launcher.increaseManualOffset();
        if (gamepad1.left_bumper && !g1_left_bumper_pressed) robot.launcher.decreaseManualOffset();

        double turret_input = -gamepad1.right_stick_x;
        if (gamepad1.dpad_right) turret_input = -1.0; 
        else if (gamepad1.dpad_left) turret_input = 1.0;

        boolean is_manually_moving_turret = Math.abs(turret_input) > 0.1;

        if (is_manually_moving_turret) {
            robot.turret.setManualControl(turret_input);
        } else if (was_manually_moving_turret) {
            robot.turret.setModeToAuto();
        }
        
        if (gamepad1.x && !g1_x_pressed) robot.turret.resetNudge();
        if (gamepad1.y && !g1_y_pressed) robot.turret.toggleAutoAim();
        if (gamepad1.a && !g1_a_pressed) robot.launcher.toggleLauncher();
        if (gamepad1.start && !g1_start_pressed) robot.driverAssist.resetHeading();

        // === Gamepad 2: Driver Controls (Intake, Diverter, Drive Mode) ===
        if (gamepad2.right_trigger > 0.1) robot.intake.run();
        else if (gamepad2.left_trigger > 0.1) robot.intake.reverse();
        else if (!actionManager.isBusy()) robot.intake.stop();

        if (gamepad2.dpad_left) actionManager.setDiverterToPurple();
        else if (gamepad2.dpad_right) actionManager.setDiverterToGreen();
        else if (gamepad2.dpad_up) actionManager.setDiverterToNeutral();
        
        if (gamepad2.b && !g2_b_pressed) {
            switch (robot.driverAssist.getMode()) {
                case ROBOT_CENTRIC: robot.driverAssist.setMode(DriverAssist.DriveMode.FIELD_CENTRIC); break;
                case FIELD_CENTRIC: robot.driverAssist.setMode(DriverAssist.DriveMode.TARGET_LOCK); break;
                case TARGET_LOCK: robot.driverAssist.setMode(DriverAssist.DriveMode.ROBOT_CENTRIC); break;
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
        
        // Gamepad 2
        g2_b_pressed = gamepad2.b;

        double turret_input = -gamepad1.right_stick_x;
        if (gamepad1.dpad_right) turret_input = -1.0; 
        else if (gamepad1.dpad_left) turret_input = 1.0;
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
