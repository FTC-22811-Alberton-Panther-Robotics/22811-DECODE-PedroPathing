package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.BallManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TurretHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "HelenaOpmode")
public class HelenaTeleOp extends OpMode {
    BallManager ballManager;
    TurretHardware turretHardware;
    DriverAssist driveHelper;
    ActionManager actionManager;
    Follower follower;
    private GameState.Alliance alliance;
    RobotHardwareContainer robot;
    MecanumHardware mecanumHardware;
    private enum TeleOpState { MANUAL}
    private TeleOpState currentState = TeleOpState.MANUAL;



    public void init() {
        // Initialize all our hardware and helper classes.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap, robot.localizer);
        driveHelper = new DriverAssist(follower);
        ballManager = new BallManager(robot);

        // --- Load State from Autonomous ---
        // This is the seamless handoff from Auto to TeleOp.
        // If the GameState class has a pose from the end of auto, set our starting position.
        if (GameState.currentPose != null) {
            follower.setPose(GameState.currentPose);
        }
        // Load the alliance color that was selected during the autonomous init phase.
        // Default to BLUE if for some reason autonomous was never run (e.g., during testing).
        this.alliance = (GameState.alliance != GameState.Alliance.UNKNOWN) ? GameState.alliance : GameState.Alliance.BLUE;

        // Provide clear instructions to the driver on the Driver Station.
        telemetry.addLine("Actions TeleOp Initialized. Ready for match!");
        telemetry.addData("Loaded Alliance", this.alliance);
        telemetry.addLine("DPAD U/L/D -> TargetLock/Field/Robot Centric");
        telemetry.addLine("X -> Override Alliance | B -> Auto-Park");
        telemetry.update();
    }

    public void loop(){
        follower.update();
        actionManager.update();
        ballManager.update();
        driveHelper.update(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, alliance);
        ManualControls();
        updateTelemetry();
    }

    private void ManualControls() {
        // gamepad 1 is for shooting and making shooter adjustments
        // gamepad 2 is for driving and intaking balls for the shooter
        // TODO: PROGRAM THE SERVOS AND MAKE DRIVERS AWARE THAT THE RIGHT
        // TODO:: DIVERTER POSTITION IS FOR GREEN AND VISE VERSA

        // BALL SHOOTER GAMEPAD 1
        if(gamepad1.rightBumperWasPressed()){
            ballManager.greenBallShoot();
        } else if (gamepad1.leftBumperWasPressed()){
            ballManager.purpleBallShoot();
        }
        // TURRET SPIN GAMEPAD 1
        if (gamepad1.right_trigger > .1){
            turretHardware.rightSpin(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > .1) {
            turretHardware.leftSpin(gamepad1.left_trigger);
        }
        // DIVERTER CODE GAMEPAD 2
        if(gamepad2.right_bumper){
            ballManager.diverterGREEN();
        }else if (gamepad2.left_bumper){
            ballManager.diverterPurple();
        }

        // INTAKE CODE GAMEPAD 2
        if (gamepad2.right_trigger > 0.1) {
            robot.intake.run();
        } else if (gamepad2.left_trigger > 0.1) {
            robot.intake.reverse();
        } else {
            robot.intake.stop();
        }
    }
    private void updateTelemetry() {
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driveHelper.getMode());
        telemetry.addData("Alliance", alliance.toString());
        // Display the robot's current field-relative position from the fused localizer.
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("H", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }



















}

