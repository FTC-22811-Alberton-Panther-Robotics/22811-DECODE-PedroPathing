package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TurretHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "HelenaOpmode")
public class HelenaTeleOp extends OpMode {
    // BallManager has been removed.
    TurretHardware turretHardware;
    DriverAssist driveHelper;
    ActionManager actionManager; // Use the new, consolidated ActionManager
    Follower follower;
    private GameState.Alliance alliance;
    RobotHardwareContainer robot;
    MecanumHardware mecanumHardware;
    private enum TeleOpState { MANUAL}
    private TeleOpState currentState = TeleOpState.MANUAL;



    public void init() {
        // Initialize all our hardware and helper classes.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot); // Initialize the new ActionManager
        follower = Constants.createFollower(hardwareMap, telemetry);
        driveHelper = new DriverAssist(follower);
        // ballManager is no longer needed.

        // --- Load State from Autonomous ---
        if (GameState.currentPose != null) {
            follower.setPose(GameState.currentPose);
        }
        this.alliance = GameState.alliance;

        telemetry.addLine("Actions TeleOp Initialized. Ready for match!");
        telemetry.addData("Loaded Alliance", this.alliance);
        telemetry.addLine("DPAD U/L/D -> TargetLock/Field/Robot Centric");
        telemetry.addLine("X -> Override Alliance | B -> Auto-Park");
        telemetry.update();
    }

    public void loop(){
        follower.update();
        actionManager.update(); // Update the new ActionManager
        driveHelper.update(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, alliance);
        ManualControls();
        updateTelemetry();
    }

    private void ManualControls() {
        // gamepad 1 is for shooting and making shooter adjustments
        // gamepad 2 is for driving and intaking balls for the shooter

        // BALL SHOOTER GAMEPAD 1 - Now using ActionManager
        if(gamepad1.rightBumperWasPressed()){
            actionManager.startGreenBallShoot();
        } else if (gamepad1.leftBumperWasPressed()){
            actionManager.startPurpleBallShoot();
        }
        // TURRET SPIN GAMEPAD 1
        if (gamepad1.right_trigger > .1){
            turretHardware.rightSpin(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > .1) {
            turretHardware.leftSpin(gamepad1.left_trigger);
        }
        // DIVERTER CODE GAMEPAD 2 - Now using ActionManager
        if(gamepad2.right_bumper){
            actionManager.setDiverterToGreen();
        }else if (gamepad2.left_bumper){
            actionManager.setDiverterToPurple();
        }

        // INTAKE CODE GAMEPAD 2
        if (gamepad2.right_trigger > 0.1) {
            robot.intake.run();
        } else if (gamepad2.left_trigger > 0.1) {
            robot.intake.reverse();
        } else {
            // Let the ActionManager handle stopping the intake if it's running a sequence.
            if (!actionManager.isBusy()) {
                robot.intake.stop();
            }
        }
    }
    private void updateTelemetry() {
        telemetry.addData("TeleOp State", currentState.toString());
        telemetry.addData("Drive Mode", driveHelper.getMode());
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("H", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
