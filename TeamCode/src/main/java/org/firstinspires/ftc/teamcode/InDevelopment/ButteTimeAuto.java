package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;


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
        intakeHardware.stop();
        launcherHardware.stop();

    }

}
