package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.BallManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DriverAssist;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.MecanumHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.Turrethardware;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "HelenaOpmode")
public class HelenaTeleOp extends OpMode {
    BallManager ballManager;
    Turrethardware turrethardware;
    DriverAssist driveHelper;
    ActionManager actionManager;
    Follower follower;
    private GameState.Alliance alliance;
    RobotHardwareContainer robot;
    MecanumHardware mecanumHardware;

    private double flywheelRPMSpeed = 2400; // Start at default speed
    private static final double RPM_ADJUST_RATE = 100; // How much to change speed by per press

    private boolean dpad_up_pressed, dpad_down_pressed, dpad_left_pressed, dpad_right_pressed;
    private boolean y_pressed, b_pressed, x_pressed, right_bumper_pressed;
    private enum TeleOpState { MANUAL, AUTO_PARK }
    private TeleOpState currentState = TeleOpState.MANUAL;



    public void init() {
        // Initialize all our hardware and helper classes.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap, robot.localizer);
        driveHelper = new DriverAssist(follower);

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
        driveHelper.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, alliance);


    }

    private void ControllerManager() {

        // gamepad 1 is for shooting and making shooter adustments
        // gamepad 2 is for driving and intaking balls for the shooter
        // TODO: PROGRAM THE SERVOS AND MAKE DRIVERS AWARE THAT THE RIGHT
        // TODO:: DIVERTER POSTITION IS FOR GREEN AND VISE VERSA

     if(gamepad1.right_bumper){
        ballManager.greenBallShoot();
     } else if (gamepad1.right_bumper){
         ballManager.purpleBallShoot();
     }
      if (gamepad1.right_trigger > .1){
          turrethardware.rightSpin();
      } else if (gamepad1.left_trigger > .1) {
          turrethardware.leftSpin();
      }


        if(gamepad2.right_trigger > .1){
         ballManager.diverterGREEN();
     }else if (gamepad2.left_trigger > .1){
         ballManager.diverterPurple();
     }





        }

}

