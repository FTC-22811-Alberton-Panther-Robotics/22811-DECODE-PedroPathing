package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;

@TeleOp(name="TeleopManualControls", group="01 Butte")
public class TeleopManualControls extends OpMode {

    RobotHardwareContainer robot;
    MecanumHardware mecanumHardware;

    private double flywheelRPMSpeed = 2400; // Start at default speed
    private static final double RPM_ADJUST_RATE = 100; // How much to change speed by per press

    private boolean dpad_up_pressed = false;
    private boolean dpad_down_pressed = false;

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        mecanumHardware = new MecanumHardware();
        mecanumHardware.init(hardwareMap);
        telemetry.addLine("Classic Controls Initialized. Ready for match!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Drivetrain ---
        mecanumHardware.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // --- Manual Intake and Transfer Controls ---
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.run();
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intake.reverse();
        } else {
            robot.intake.stop();
        }

        if (gamepad1.y) {
            robot.transfer.run();
        } else if (gamepad1.a) {
            robot.transfer.reverse();
        } else {
            robot.transfer.stop();
        }

        // --- Manual Launcher Controls with Adjustable Speed ---
        if (gamepad1.right_bumper) {
            // Use the new spinUp method that takes our adjustable variable
            robot.launcher.spinUp(flywheelRPMSpeed);
        } else if (gamepad1.left_bumper) {
            robot.launcher.reverse();
        } else {
            robot.launcher.stop();
        }

        // --- Adjustable Flywheel Speed ---
        if (gamepad1.dpad_up && !dpad_up_pressed) {
            flywheelRPMSpeed += RPM_ADJUST_RATE;
        } else if (gamepad1.dpad_down && !dpad_down_pressed) {
            flywheelRPMSpeed -= RPM_ADJUST_RATE;
        }

        dpad_up_pressed = gamepad1.dpad_up;
        dpad_down_pressed = gamepad1.dpad_down;

        telemetry.addData("Mode", "Classic Controls");
        telemetry.addData("Target RPM", "%.0f", flywheelRPMSpeed);
        telemetry.addData("Actual RPM", "%.0f", robot.launcher.getLeftFlywheelRPM());
        telemetry.update();
    }
}
