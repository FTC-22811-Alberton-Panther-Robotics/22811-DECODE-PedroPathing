package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ActionManager is the definitive, unified class for controlling all robot mechanisms.
 * It is built as a state machine to handle complex, multi-step sequences like shooting and intaking.
 */
public class ActionManager {

    private final RobotHardwareContainer robot;
    private final ElapsedTime timer = new ElapsedTime();

    public enum ActionState {
        IDLE,
        INTAKING,
        SHOOT_TRANSFER_LEFT,
        SHOOT_TRANSFER_RIGHT,
        SHOOT_SCOOP,
        WAITING_FOR_AUTO_ACTION // A generic state for simple, timed auto commands
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robot) {
        this.robot = robot;
    }

    /**
     * This is the heart of the manager. It must be called in every loop of an OpMode.
     */
    public void update() {
        switch (currentState) {
            case IDLE:
                break;

            case WAITING_FOR_AUTO_ACTION:
                if (timer.seconds() > 1.5) { // Default time for simple auto actions
                    stopAll();
                }
                break;

            case INTAKING:
                if (timer.seconds() > 0.5) { // Run intake for a short burst
                    robot.intake.stop();
                    currentState = ActionState.IDLE;
                }
                break;

            // CORRECTED: Increased timer to allow servos to complete their full stroke.
            case SHOOT_TRANSFER_LEFT:
                if (timer.seconds() > 1.0) {
                    robot.transfer.LeftTransferReturn();
                    currentState = ActionState.SHOOT_SCOOP;
                    timer.reset();
                }
                break;

            case SHOOT_TRANSFER_RIGHT:
                if (timer.seconds() > 1.0) {
                    robot.transfer.RightTransferReturn();
                    currentState = ActionState.SHOOT_SCOOP;
                    timer.reset();
                }
                break;

            case SHOOT_SCOOP:
                if (timer.seconds() > 0.67) {
                    robot.scoop.ballDown();
                    currentState = ActionState.IDLE;
                }
                break;
        }
    }

    // ----- PUBLIC METHODS FOR TELEOP AND AUTO -----

    public void startGreenBallShoot() {
        if (isBusy()) return;
        currentState = ActionState.SHOOT_TRANSFER_RIGHT;
        robot.transfer.runRight();
        robot.scoop.ballUp();
        timer.reset();
    }

    public void startPurpleBallShoot() {
        if (isBusy()) return;
        currentState = ActionState.SHOOT_TRANSFER_LEFT;
        robot.transfer.runLeft();
        robot.scoop.ballUp();
        timer.reset();
    }

    public void setDiverterToGreen() {
        robot.diverter.setPosition(DiverterHardware.GatePosition.GREEN);
    }

    public void setDiverterToPurple() {
        robot.diverter.setPosition(DiverterHardware.GatePosition.PURPLE);
    }

    public void setDiverterToNeutral() {
        robot.diverter.setPosition(DiverterHardware.GatePosition.NEUTRAL);
    }

    // Generic actions for the autonomous playlist
    public void startIntake() {
        if(isBusy()) return;
        currentState = ActionState.WAITING_FOR_AUTO_ACTION;
        robot.intake.run();
        timer.reset();
    }

    public void startLaunch() {
        if (isBusy()) return;
        currentState = ActionState.WAITING_FOR_AUTO_ACTION;
        timer.reset();
    }

    /**
     * A specific, controlled action to clear pixel jams.
     * It reverses the intake and sets the diverter to a neutral position.
     */
    public void clearJam() {
        robot.intake.reverse();
        robot.diverter.setNeutral();
    }

    public void startSmartLaunch(GameState.ObeliskPattern pattern) {
        if (isBusy()) return;
        startPurpleBallShoot(); 
    }

    public void stopAll() {
        robot.intake.stop();
        robot.launcher.stop();
        robot.transfer.LeftTransferReturn();
        robot.transfer.RightTransferReturn();
        robot.scoop.ballDown();
        currentState = ActionState.IDLE;
    }

    public boolean isBusy() {
        return currentState != ActionState.IDLE;
    }
}
