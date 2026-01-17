package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "HelenaTeleOp")
public class HelenaTeleOp extends OpMode {

    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private Follower follower;

    private GameState.Alliance alliance;

    // Button press trackers for edge detection
    private boolean g1_dpad_left_pressed, g1_dpad_right_pressed, g1_y_pressed, g1_x_pressed, 
                    g1_right_bumper_pressed, g1_left_bumper_pressed, g1_a_pressed, g1_joystick_was_moving;

    @Override
    public void init() {
        // The RobotHardwareContainer now correctly creates and initializes all hardware,
        // including the mecanum drive.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap, robot);
        robot.initTurret(follower, hardwareMap); 
        robot.initLauncher(follower, robot.turret, hardwareMap);
        
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

        // CORRECTED: Driving is now handled by the simple and reliable MecanumHardware class.
        robot.mecanumDrive.drive(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x);

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

        boolean joystick_moving = Math.abs(gamepad1.right_stick_x) > 0.1;
        if (joystick_moving) {
            robot.turret.setManualPower(-gamepad1.right_stick_x); // Inverted for intuitive control
        } else if (g1_joystick_was_moving) {
            robot.turret.setModeToAuto();
        }
        
        if (gamepad1.dpad_right && !g1_dpad_right_pressed) robot.turret.adjustNudge(5.0);
        if (gamepad1.dpad_left && !g1_dpad_left_pressed) robot.turret.adjustNudge(-5.0);
        
        if (gamepad1.x && !g1_x_pressed) robot.turret.resetNudge();
        if (gamepad1.y && !g1_y_pressed) robot.turret.toggleAutoAim();
        if (gamepad1.a && !g1_a_pressed) robot.launcher.toggleLauncher();

        // === Gamepad 2: Driver Controls (Intake, Diverter) ===
        if (gamepad2.right_trigger > 0.1) robot.intake.run();
        else if (gamepad2.left_trigger > 0.1) robot.intake.reverse();
        else if (!actionManager.isBusy()) robot.intake.stop();

        if (gamepad2.dpad_left) actionManager.setDiverterToPurple();
        else if (gamepad2.dpad_right) actionManager.setDiverterToGreen();
        else if (gamepad2.dpad_up) actionManager.setDiverterToNeutral();
    }

    private void updateButtonStates() {
        // Gamepad 1
        g1_dpad_left_pressed = gamepad1.dpad_left;
        g1_dpad_right_pressed = gamepad1.dpad_right;
        g1_y_pressed = gamepad1.y;
        g1_x_pressed = gamepad1.x;
        g1_a_pressed = gamepad1.a;
        g1_right_bumper_pressed = gamepad1.right_bumper;
        g1_left_bumper_pressed = gamepad1.left_bumper;
        g1_joystick_was_moving = Math.abs(gamepad1.right_stick_x) > 0.1;
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Turret Auto-Aim", robot.turret.isAutoAimActive ? "ACTIVE" : "OFF");
        telemetry.addData("Launcher Target RPM", "%.0f", robot.launcher.getCurrentTargetVelocity());
        telemetry.addData("Launcher State", robot.launcher.isLauncherOn() ? "ON" : "OFF");

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
