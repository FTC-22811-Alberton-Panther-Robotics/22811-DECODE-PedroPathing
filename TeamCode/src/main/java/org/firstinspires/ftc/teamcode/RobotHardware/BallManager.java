package org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.ACTION_COMPLETE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.GREEN_BALL_INTAKE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.PURPLE_BALL_INTAKE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.PURPLE_BALL_TRANSFER_LEFT;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.GREEN_BALL_TRANSFER_RIGHT;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.IDLE;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BallManager {
    private final RobotHardwareContainer robot;
    private ScooperHardware scooperHardware;
    private BallTransferHardware ballTransfer;
    private final IntakeHardware intake;
    private final BallDiverterHardware BallDiverterHardware;
    private final LauncherHardware launcher;
    private final TransferHardware transfer;
    private final ElapsedTime actionTimer = new ElapsedTime();


    public BallManager(RobotHardwareContainer robotContainer) {
        this.robot = robotContainer;
        this.intake = robot.intake;
        this.launcher = robot.launcher;
        this.transfer = robot.transfer;
        this.BallDiverterHardware = robot.ballDiverter;
    }

    public enum BallActions {
        IDLE,
        ACTION_COMPLETE,
        GREEN_BALL_TRANSFER_RIGHT,
        PURPLE_BALL_TRANSFER_LEFT,
        BALL_SCOOP,
        GREEN_BALL_INTAKE,
        PURPLE_BALL_INTAKE,
    }
    private  BallActions  currentState = IDLE;

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE:
                break; // No timed logic in these states


            case GREEN_BALL_INTAKE:
                BallDiverterHardware.greenBall();
                intake.run();
                if (actionTimer.seconds() > .5){
                   intake.stop();
                   currentState = ACTION_COMPLETE;
                }


            case PURPLE_BALL_INTAKE:
                 BallDiverterHardware.purpleBall();
                 intake.run();
                if (actionTimer.seconds() > .5){
                    intake.stop();
                    currentState = ACTION_COMPLETE;
                }

            case PURPLE_BALL_TRANSFER_LEFT:
                ballTransfer.leftTransfer();
                if (actionTimer.seconds() > .5) {
                    ballTransfer.leftReturn();
                    currentState = BALL_SCOOP;
                }
                break;

            case GREEN_BALL_TRANSFER_RIGHT:
                ballTransfer.rightTransfer();
                if(actionTimer.seconds() > .5){
                    ballTransfer.rightReturn();
                    currentState = BALL_SCOOP;
                }
                break;

            case BALL_SCOOP:
                scooperHardware.ballUp();
                if (actionTimer.seconds() > .67) {
                    scooperHardware.ballDown();
                    currentState = ACTION_COMPLETE;
                }
                break;



        }
    }

    public void greenBallShoot(){
        if (isBusy()) return;
        currentState = GREEN_BALL_TRANSFER_RIGHT;
    }
    public void purpleBallShoot(){
        if (isBusy()) return;
        currentState = PURPLE_BALL_TRANSFER_LEFT;
    }

    public void diverterPurple(){
        if (isBusy()) return;
        currentState = PURPLE_BALL_INTAKE;


    }

    public void diverterGREEN(){
        if (isBusy()) return;
        currentState = GREEN_BALL_INTAKE;
    }

    // ----- Public Methods to Trigger Actions -----
    public boolean isBusy() {
        return currentState != BallActions.IDLE && currentState != BallActions.ACTION_COMPLETE;
    }

    public void completeAction() {
        currentState = BallActions.IDLE;
    }

    public void stopAll() {
        intake.stop();
        launcher.stop();
        transfer.stop();
        currentState = BallActions.IDLE;
    }
}
