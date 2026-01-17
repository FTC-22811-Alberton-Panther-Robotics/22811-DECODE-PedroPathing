package org.firstinspires.ftc.teamcode.InDevelopment;

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

@TeleOp(name = "TeleopWithActions", group = "01 Helena")
public class TeleopWithActions extends OpMode {

    RobotHardwareContainer robot;
    ActionManager actionManager;
    Follower follower;
    DriverAssist driveHelper;

    private enum TeleOpState { MANUAL, AUTO_PARK }
    private TeleOpState currentState = TeleOpState.MANUAL;

    private GameState.Alliance alliance;

    // CORRECTED: Updated button trackers for the new control scheme
    private boolean y_pressed, b_pressed, x_pressed, right_bumper_pressed, back_pressed;
    private DriverAssist.DriveMode previousDriveMode = DriverAssist.DriveMode.ROBOT_CENTRIC;

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap, robot);
        robot.initTurret(follower, hardwareMap);
        robot.initLauncher(follower, robot.turret, hardwareMap);

        actionManager = new ActionManager(robot);
        driveHelper = new DriverAssist(follower);

        if (GameState.currentPose != null) {
            follower.setStartingPose(GameState.currentPose);
        } else {
            follower.setStartingPose(new Pose());
        }
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        telemetry.addLine("Actions TeleOp Initialized. Ready for match!");
        telemetry.addLine("X: Toggle Robot/Field Centric | BACK: Toggle Target Lock");
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
        follower.update();
        actionManager.update();
        robot.turret.update(alliance);
        robot.launcher.update();

        switch (currentState) {
            case MANUAL:
                // CORRECTED: New drive mode selection logic
                if (gamepad1.x && !x_pressed) {
                    if (driveHelper.getMode() == DriverAssist.DriveMode.FIELD_CENTRIC) {
                        driveHelper.setMode(DriverAssist.DriveMode.ROBOT_CENTRIC);
                    } else {
                        driveHelper.setMode(DriverAssist.DriveMode.FIELD_CENTRIC);
                    }
                }

                if (gamepad1.back && !back_pressed) {
                    if (driveHelper.getMode() != DriverAssist.DriveMode.TARGET_LOCK) {
                        previousDriveMode = driveHelper.getMode();
                        driveHelper.setMode(DriverAssist.DriveMode.TARGET_LOCK);
                    } else {
                        driveHelper.setMode(previousDriveMode);
                    }
                }

                try {
                    driveHelper.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, alliance);
                } catch (Exception e) {
                    // Safely ignore first-frame error
                }

                if (gamepad1.b && !b_pressed) {
                    Pose parkPose = (alliance == GameState.Alliance.BLUE) ? FieldPosePresets.BLUE_BASE : FieldPosePresets.RED_BASE;
                    follower.followPath(new Path(new BezierLine(follower.getPose(), parkPose)));
                    currentState = TeleOpState.AUTO_PARK;
                }

                handleManualMechanisms();
                break;

            case AUTO_PARK:
                telemetry.addLine("--- AUTO-PARKING --- ");
                if (!follower.isBusy() || Math.abs(gamepad1.left_stick_y) > 0.1) {
                    follower.breakFollowing();
                    currentState = TeleOpState.MANUAL;
                }
                break;
        }

        updateButtonStates();
        updateTelemetry();
    }

    private void handleManualMechanisms() {
        if (gamepad1.right_trigger > 0.1) actionManager.startLaunch();
        else if (gamepad1.right_bumper && !right_bumper_pressed) actionManager.startIntake();
        else if (gamepad1.left_bumper) actionManager.clearJam();
        else if (gamepad1.y && !y_pressed) robot.intake.run();
        else if (!actionManager.isBusy()) actionManager.stopAll();
    }

    private void updateButtonStates() {
        right_bumper_pressed = gamepad1.right_bumper;
        y_pressed = gamepad1.y;
        b_pressed = gamepad1.b;
        x_pressed = gamepad1.x;
        back_pressed = gamepad1.back;
    }

    private void updateTelemetry() {
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driveHelper.getMode());
        telemetry.addData("Alliance", alliance.toString());
        
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            telemetry.addData("X", "%.2f", currentPose.getX());
            telemetry.addData("Y", "%.2f", currentPose.getY());
            telemetry.addData("H", "%.1f", Math.toDegrees(currentPose.getHeading()));
        } else {
            telemetry.addLine("Pose: Initializing...");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        actionManager.stopAll();
        follower.breakFollowing();
    }
}
