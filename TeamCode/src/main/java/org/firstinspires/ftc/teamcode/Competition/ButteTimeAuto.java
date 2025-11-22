package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;


@Autonomous(name="ButteTimeAuto", group="01 Butte", preselectTeleOp="Butte2Controller")
public class ButteTimeAuto extends OpMode{
    LauncherHardware launcherHardware = new LauncherHardware();
    IntakeHardware intakeHardware = new IntakeHardware();
    MecanumHardware mecanumHardware = new MecanumHardware();
    TransferHardware transferHardware = new TransferHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
        launcherHardware.init(hardwareMap);
        intakeHardware.init(hardwareMap);
        mecanumHardware.init(hardwareMap);
        transferHardware.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        // Drive forward for 2 seconds
        while (runtime.seconds() < 2) {

            mecanumHardware.drive(.5, 0, 0);
            telemetry.addData("Status", "Autonomous Running");
            telemetry.update();

            if(runtime.seconds()> 10){
                break;
            }
        }
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Autonomous Finished");
        telemetry.update();
    }

    @Override
    public void stop() {
        mecanumHardware.stop();
        transferHardware.stop();
        intakeHardware.stop();
        launcherHardware.stop();

    }

}
