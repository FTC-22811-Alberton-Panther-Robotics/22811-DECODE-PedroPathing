//package org.firstinspires.ftc.teamcode.InDevelopment;
//
//import org.firstinspires.ftc.teamcode.RobotHardware.intakeHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;
//import org.firstinspires.ftc.teamcode.RobotHardware.TurretHardware;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp(name ="HelenaTeleop")
//public class TurretControllerControllTeleop extends OpMode{
//
//
//    TurretHardware turretHardware = new TurretHardware();
//    LauncherHardware launcherHardware = new LauncherHardware();
//    intakeHardware intakeHardware = new intakeHardware();
//    MecanumHardware mecanumHardware = new MecanumHardware();
//    TransferHardware transferHardware = new TransferHardware();
//
//
//
//
//
//    @Override
//    public void init() {
//        turretHardware.init(hardwareMap);
//        launcherHardware.init(hardwareMap);
//        intakeHardware.init(hardwareMap);
//        mecanumHardware.init(hardwareMap);
//        transferHardware.init(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//
//        /// ** driving and intake on gampead 1 **///
//        double forward = -gamepad1.left_stick_y;  // Controls forward and backward movement
//        double strafe = gamepad1.left_stick_x;   // Controls side-to-side movement
//        double turn = gamepad1.right_stick_x;    // Controls turning/rotation
//        mecanumHardware.drive(forward, strafe, turn);
//
//        // Intake
//        if (gamepad1.dpad_right){
//            intakeHardware.run();
//        }else if (gamepad1.dpad_left){
//            intakeHardware.reverse();
//        }else if (gamepad1.dpad_down){
//            intakeHardware.run();
//
//        }
//
//        /// ** Shooting and intake/transfer on gamepad 2 ** ///
//        // spins the turrethead around
//        if(gamepad2.right_trigger > .1){
//            turretHardware.rightSpin();
//        } else if (gamepad2.left_trigger > .1) {
//            turretHardware.leftSpin();
//        }else {
//            turretHardware.stop();
//        }
//
//        // runs the intake/transfer system into the turret this is based off of our current design 12/10/25
//        if (gamepad2.dpad_right){
//            intakeHardware.run();
//        }else if (gamepad2.dpad_left){
//            intakeHardware.reverse();
//        }else if (gamepad2.dpad_down){
//            intakeHardware.run();
//
//        }
//
//
//
//    }
//
//
//    public void stop(){
//        transferHardware.stop();
//        intakeHardware.stop();
//        launcherHardware.stop();
//        turretHardware.stop();
//
//    }
//}
