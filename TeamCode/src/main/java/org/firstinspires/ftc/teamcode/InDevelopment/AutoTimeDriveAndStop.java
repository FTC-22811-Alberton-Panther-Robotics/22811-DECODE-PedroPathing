package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;


@Autonomous(name="AutoTimeDriveAndStop", group="Bozeman", preselectTeleOp="BozemanTeleop")
public class AutoTimeDriveAndStop extends OpMode{
    //LauncherHardware launcherHardware = new LauncherHardware();
    //intakeHardware intakeHardware = new intakeHardware();
    MecanumHardware mecanumHardware = new MecanumHardware();
    TransferHardware transferHardware = new TransferHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(){
//          LauncherHardware.init(hardwareMap);
//        intakeHardware.init(hardwareMap);
        mecanumHardware.init(hardwareMap);
        transferHardware.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (runtime.seconds() < 5){
            mecanumHardware.drive(.5, 0, 0);
        }
        telemetry.addData("Status", "Autonomous Running");
        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Autonomous Finished");
        telemetry.update();
        mecanumHardware.stop();
        transferHardware.stop();
//        RintakeHardware.stop();
//        launcherHardware.stop();

    }

}
