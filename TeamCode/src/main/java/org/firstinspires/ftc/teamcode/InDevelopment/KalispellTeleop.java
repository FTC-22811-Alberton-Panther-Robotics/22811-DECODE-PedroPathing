package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;

@TeleOp(name="KalispellTeleop", group="01 Kalispell")
@Disabled
public class KalispellTeleop extends OpMode {
    LauncherHardware launcherHardware = new LauncherHardware();
    IntakeHardware intakeHardware = new IntakeHardware();
    MecanumHardware mecanumHardware = new MecanumHardware();
    TransferHardware transferHardware = new TransferHardware();
    public boolean shootersOn = false;
    private boolean aButtonPressed = false;
    public double targetShooterSpeed_RPM = 2000;
    public double closetargetShooterSpeed_RPM = 2000;
    public double farTargetShooterSpeed_RPM = 2000;

    private ElapsedTime runtime = new ElapsedTime();
    private double leftBumperLastTime = 0;
    private double rightBumperLastTime = 0;
    private static int SHOOTER_SPEED_INCREMENT = 100;

    @Override
    public void init(){
        launcherHardware.init(hardwareMap);
        intakeHardware.init(hardwareMap);
        mecanumHardware.init(hardwareMap);
        transferHardware.init(hardwareMap);
    }

    @Override
    public void loop(){
        ///  potentially adding another controller
        ///  one for driving and intake, another for shooting and transfer system possibly
            // Driving
            double forward = -gamepad1.left_stick_y;  // Controls forward and backward movement
            double strafe = gamepad1.left_stick_x;   // Controls side-to-side movement
            double turn = gamepad1.right_stick_x;    // Controls turning/rotation
            mecanumHardware.drive(forward, strafe, turn);
            // Intake
            if (gamepad1.dpad_right){
                intakeHardware.run();
            }else if (gamepad1.dpad_left){
                intakeHardware.reverse();
            }else if (gamepad1.dpad_down){
                intakeHardware.run();

            }
            // Launcher flywheel speed
            if (gamepad1.right_bumper) {
                if (runtime.seconds() - rightBumperLastTime > .075) {
                    if (targetShooterSpeed_RPM < LauncherHardware.MAX_RPM) {
                        targetShooterSpeed_RPM += SHOOTER_SPEED_INCREMENT;
                    }
                    rightBumperLastTime = runtime.seconds();
                }
            } else if (gamepad1.left_bumper) {
                if (runtime.seconds() - leftBumperLastTime > .075) {
                    if (targetShooterSpeed_RPM > 0) {
                        targetShooterSpeed_RPM -= SHOOTER_SPEED_INCREMENT;
                    }
                    leftBumperLastTime = runtime.seconds();
                }
            }

            // reverses launchers hopefully

            if(gamepad1.b) {
                launcherHardware.reverse();
            }else{
                launcherHardware.reverse();
            }

            // Toggle shooter
            if (gamepad1.a) {
                if (!aButtonPressed) {
                    aButtonPressed = true;
                    shootersOn = !shootersOn;
                }
            } else {
                aButtonPressed = false;
            }


            if (shootersOn) {
                launcherHardware.spinUp(targetShooterSpeed_RPM);
            } else {
                launcherHardware.spinUp(0);
            }

            // Ball transfer
            if(gamepad1.right_trigger > 0.1){
                transferHardware.run();
            }else if (gamepad1.left_trigger > 0.1){
                transferHardware.reverse();
            } else{
                transferHardware.stop();
            }
            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Launcher Target Speed", targetShooterSpeed_RPM);
            telemetry.addData("LeftFlywheel RPM",  "%.0f", launcherHardware.getLeftFlywheelRPM());
            telemetry.addData("Close Triangle Target RPM",  "%.0f", closetargetShooterSpeed_RPM);
            telemetry.addData("Far Triangle Target RPM",  "%.0f", farTargetShooterSpeed_RPM);

        telemetry.update();
    }

    public void stop(){
        transferHardware.stop();
        intakeHardware.stop();
        launcherHardware.stop();
    }

}



