package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.RobotHardware.ScoopHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Launcher Speed Tuning", group = "Tuning")
public class LauncherTuningOpMode extends OpMode {
    private Follower follower;
    private CombinedLocalizer localizer;
    private RobotHardwareContainer robot;
    private ActionManager actionManager;
    private LauncherHardware launcher;
    private IntakeHardware intake;
    private TransferHardware transfer;
    private ScoopHardware scoop;

    // --- Tuning Variables ---
    private double targetRPM = 3000.0; // Starting RPM
    private final double RPM_INCREMENT = 20.0; // Adjust RPM by this much
    private boolean launcherOn = false;

    @Override
    public void init() {

        localizer = new CombinedLocalizer(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        // We don't need a follower for this tuning OpMode
        actionManager = new ActionManager(robot) ;
        robot.initLauncher(follower, hardwareMap);
        telemetry.addLine("Launcher Tuning OpMode Initialized.");
        telemetry.addLine("A: Toggle Launcher | D-Pad U/D: Adjust RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        actionManager.update();
        handleLauncherControls();
        handleMechanismControls();
        updateTelemetry();
    }

    private void handleLauncherControls() {
        if (gamepad1.aWasPressed()) {
            launcherOn = !launcherOn;
        }
        if (launcherOn) {
            robot.launcher.setLaunchSpeed(targetRPM);
        } else {
            robot.launcher.stop();
        }
        if (launcherOn) {
            if (gamepad1.dpadUpWasPressed()) {
                targetRPM += RPM_INCREMENT;
                robot.launcher.setLaunchSpeed(targetRPM);
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetRPM -= RPM_INCREMENT;
                robot.launcher.setLaunchSpeed(targetRPM);
            }
        }
    }

    private void handleMechanismControls() {
        // --- Intake ---
        if (gamepad1.left_trigger > .1) {
            intake.run();
        } else {
            intake.stop();
        }

        // --- Transfer/Kicker ---
        if (gamepad1.leftBumperWasPressed()) {
            actionManager.startTransferGreenArtifact();

        }
        if (gamepad1.rightBumperWasPressed()) {
            actionManager.startTransferPurpleArtifact();
        }

        // --- Scoop ---
        if (gamepad1.right_trigger > .1) {
            scoop.up();
        } else {
            scoop.down();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Launcher State", launcherOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine("------------");
        telemetry.addData("Left Flywheel RPM", "%.0f", robot.launcher.getLeftFlywheelRPM());
        telemetry.addData("Right Flywheel RPM", "%.0f", robot.launcher.getRightFlywheelRPM());
        telemetry.addLine("------------");
        telemetry.addData("Left Flywheel Current (A)", "%.2f", robot.launcher.getLeftFlywheelCurrent());
        telemetry.addData("Right Flywheel Current (A)", "%.2f", robot.launcher.getRightFlywheelCurrent());
        telemetry.update();
    }
}
