package org.firstinspires.ftc.teamcode.InDevelopment;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.BallManager;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;

@TeleOp(name = "HelenaOpmode")
public class HelenaTeleOp {
    BallManager ballManager;
    ActionManager actionManager;

    RobotHardwareContainer robot;
    MecanumHardware mecanumHardware;

    private double flywheelRPMSpeed = 2400; // Start at default speed
    private static final double RPM_ADJUST_RATE = 100; // How much to change speed by per press

    private boolean dpad_up_pressed = false;
    private boolean dpad_down_pressed = false;

    public void init() {
        actionManager = new ActionManager(robot);
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        mecanumHardware = new MecanumHardware();
        mecanumHardware.init(hardwareMap);
        telemetry.addLine("Classic Controls Initialized. Ready for match!");
        telemetry.update();
    }

    public void loop(){


    }
}
