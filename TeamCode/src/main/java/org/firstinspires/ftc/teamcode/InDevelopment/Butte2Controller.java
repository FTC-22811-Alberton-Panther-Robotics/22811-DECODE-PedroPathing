//package org.firstinspires.ftc.teamcode.InDevelopment;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.RobotHardware.BallManager;
//import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
//@Disabled
//@TeleOp(name="Butte2Controller", group="01 Butte")
//public class Butte2Controller extends OpMode {
//
//    RobotHardwareContainer robot;
//    BallManager ballManager;
//    MecanumHardware mecanumHardware;
//    LauncherHardware launcherHardware;
//    private double flywheelRPMSpeed = 2400; // Start at default speed
//    private static final double RPM_ADJUST_RATE = 100; // How much to change speed by per press
//
//    private boolean dpad_up_pressed = false;
//    private boolean dpad_down_pressed = false;
//
//    @Override
//    public void init() {
//        robot = new RobotHardwareContainer(hardwareMap, telemetry);
//        mecanumHardware = new MecanumHardware();
//        mecanumHardware.init(hardwareMap);
//        telemetry.addLine("Classic Controls Initialized. Ready for match!");
//        telemetry.update();
//    }
//
//    @Override
//    public void loop() {
//        // --- Drivetrain ---
//        mecanumHardware.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//
//        // --- Manual Intake and Transfer Controls ---
//        if (gamepad2.right_trigger >.1) {
//            robot.intake.run();
//        } else if (gamepad2.left_trigger > .1) {
//            robot.intake.reverse();
//        } else {
//            robot.intake.stop();
//        }
//
//        if (gamepad1.right_trigger > .1) {
////            robot..run();
////        } else if (gamepad1.left_trigger > .1) {
////            robot.transfer.reverse();
////        } else {
////            robot.transfer.stop();
////        }
//
//        // --- Manual Launcher Controls with Adjustable Speed ---
//        if (gamepad1.right_bumper) {
//            // Use the new spinUp method that takes our adjustable variable
//            robot.launcher.spinUp(flywheelRPMSpeed);
//        } else if (gamepad1.left_bumper) {
//            robot.launcher.reverse();
//        } else {
//            robot.launcher.stop();
//        }
//
//        // --- Adjustable Flywheel Speed ---
//        if (gamepad1.dpad_up && !dpad_up_pressed) {
//            flywheelRPMSpeed += RPM_ADJUST_RATE;
//        } else if (gamepad1.dpad_down && !dpad_down_pressed) {
//            flywheelRPMSpeed -= RPM_ADJUST_RATE;
//        }
//
//
//        dpad_up_pressed = gamepad1.dpad_up;
//        dpad_down_pressed = gamepad1.dpad_down;
//
//        telemetry.addData("Mode", "Classic Controls");
//        telemetry.addData("Target RPM", "%.0f", flywheelRPMSpeed);
//        telemetry.addData("Actual RPM", "%.0f", robot.launcher.getLeftFlywheelRPM());
//        telemetry.update();
//    }
//}
