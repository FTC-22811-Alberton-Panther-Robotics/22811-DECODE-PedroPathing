package org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.ACTION_COMPLETE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.IDLE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.BallDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.BallTransfer;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.Scooperhardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;

public class BallManager {
    private final RobotHardwareContainer robot;
    private final Scooperhardware scooperHardware;
    private final BallTransfer ballTransfer;
    private final IntakeHardware intake;
    private final BallDiverterHardware BallDiverterHardware;
    private final LauncherHardware launcher;
    private final TransferHardware transfer;
    private final ElapsedTime actionTimer = new ElapsedTime();


    public BallManager(RobotHardwareContainer robotContainer, Scooperhardware scooperHardware, BallTransfer ballTransfer) {
        this.robot = robotContainer;
        this.scooperHardware = scooperHardware;
        this.ballTransfer = ballTransfer;
        this.intake = robot.intake;
        this.launcher = robot.launcher;
        this.transfer = robot.transfer;
        this.BallDiverterHardware = robot.ballDiverter;
    }

    public enum BallActions {
        IDLE,
        ACTION_COMPLETE,
        BALL_TRANSFER_RIGHT,
        BALL_TRANSFER_LEFT,
        BALL_SCOOP,
    }
    private  BallActions  currentState = IDLE;

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE:
                break; // No timed logic in these states


            case BALL_TRANSFER_LEFT:
                ballTransfer.leftTransfer();
                if (actionTimer.seconds() > .5) {
                    ballTransfer.leftReturn();
                    currentState = BALL_SCOOP;
                }
                break;

            case BALL_TRANSFER_RIGHT:
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
