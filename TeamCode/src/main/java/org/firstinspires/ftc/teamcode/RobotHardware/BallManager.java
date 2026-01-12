package org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.ACTION_COMPLETE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_DIVERTER_GREEN;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP_RESET;
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

        REVERSING,
        ACTION_COMPLETE,
        BALL_TRANSFER_RIGHT,
        BALL_TRANSFER_LEFT,
        BALL_SCOOP,
        BALL_SCOOP_RESET,
        BALL_DIVERTER_PURPLE,
        BALL_DIVERTER_GREEN,


    }
    private  BallActions  currentState = IDLE;




    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE: case REVERSING:
                break; // No timed logic in these states


            case BALL_TRANSFER_LEFT:
                if (actionTimer.seconds() > .5) {
                    ballTransfer.leftRun();
                    currentState = BALL_SCOOP;
                }
                break;

            case BALL_TRANSFER_RIGHT:
                if(actionTimer.seconds() > .5){
                    currentState = BALL_DIVERTER_GREEN;
                    ballTransfer.rightRun();
                    currentState = BALL_SCOOP;
                }
                break;

            case BALL_SCOOP:
                scooperHardware.ballUp();
                currentState = BALL_SCOOP_RESET;
                break;

            case BALL_SCOOP_RESET:
                scooperHardware.ballDown();
                currentState = ACTION_COMPLETE;
                break;


            case BALL_DIVERTER_GREEN:
                BallDiverterHardware.greenBall();

                break;

            case BALL_DIVERTER_PURPLE:

                BallDiverterHardware.purpleBall();
            break;

        }
    }

    // ----- Public Methods to Trigger Actions -----


    public void reverseAll() {
        currentState = BallActions.REVERSING;
        intake.reverse();
        launcher.reverse();
        transfer.reverse();
    }

    // --- New Safe Methods for Color Diverter ---


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
