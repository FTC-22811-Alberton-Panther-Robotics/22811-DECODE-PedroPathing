package org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.ACTION_COMPLETE;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_DIVERTER_GREEN;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.BALL_SCOOP_RESET;
import static org.firstinspires.ftc.teamcode.RobotHardware.BallManager.BallActions.IDLE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.BallDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.BallTransfer;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.ColorDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.Scooperhardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;

public class BallManager {

    private final RobotHardwareContainer robot;

    private final Scooperhardware scooperHardware;
    private final BallTransfer BallTransfer;
    private final IntakeHardware intake;
    private final BallDiverterHardware BallDiverterHardware;
    private final LauncherHardware launcher;
    private final TransferHardware transfer;
    // The ColorDiverterHardware is nullable, as it may not exist on all robot configurations.
    private final ColorDiverterHardware colorDiverter;
    private final ElapsedTime actionTimer = new ElapsedTime();

    public BallManager(RobotHardwareContainer robot, Scooperhardware scooperhardware, BallTransfer ballTransfer, IntakeHardware intake, BallDiverterHardware ballDiverterHardware, LauncherHardware launcher, TransferHardware transfer, ColorDiverterHardware colorDiverter) {
        this.robot = robot;
        this.scooperHardware = scooperhardware;
        this.BallTransfer = ballTransfer;
        this.intake = intake;
        BallDiverterHardware = ballDiverterHardware;
        this.launcher = launcher;
        this.transfer = transfer;
        this.colorDiverter = colorDiverter;
    }

    public enum BallActions {
        IDLE,
        INTAKING,
        LAUNCHING_SPINUP,
        LAUNCHING_FIRE,
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

            case INTAKING:
                if (actionTimer.seconds() > 2.5) { // Tune this time
                    stopAll();
                    currentState = ACTION_COMPLETE;
                }
                break;


            case BALL_TRANSFER_LEFT:
                if (actionTimer.seconds() > .5) {
                    BallTransfer.leftRun();
                    currentState = BALL_SCOOP;
                }
                break;

            case BALL_TRANSFER_RIGHT:
                if(actionTimer.seconds() > .5){
                    currentState = BALL_DIVERTER_GREEN;
                    BallTransfer.rightRun();
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

    public void startIntake() {
        if (isBusy()) return;
        currentState = BallActions.INTAKING;
        intake.run();
        actionTimer.reset();
    }

    public void startLaunch() {
        if (isBusy()) return;
        currentState = BallActions.LAUNCHING_SPINUP;
        launcher.spinUp();
    }

    public void reverseAll() {
        currentState = BallActions.REVERSING;
        intake.reverse();
        launcher.reverse();
        transfer.reverse();
    }

    // --- New Safe Methods for Color Diverter ---

    /** Safely sets the diverter gate to the PURPLE position. */
    public void setDiverterPurple() {
        if (colorDiverter != null) {
            colorDiverter.setPosition(ColorDiverterHardware.GatePosition.PURPLE);
        }
    }

    /** Safely sets the diverter gate to the GREEN position. */
    public void setDiverterGreen() {
        if (colorDiverter != null) {
            colorDiverter.setPosition(ColorDiverterHardware.GatePosition.GREEN);
        }
    }

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
