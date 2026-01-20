package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ActionManager is the definitive, unified class for controlling all robot mechanisms.
 * It is built as a state machine to handle complex, multi-step sequences like shooting and intaking.
 */
public class ActionManager {

    private final RobotHardwareContainer robot;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean scoopHasBeenMoved = false;

    public enum ActionState {
        IDLE,
        INTAKING,
        TRANSFER_GREEN_TO_SCOOP,
        TRANSFER_PURPLE_TO_SCOOP,
        LAUNCH_FROM_SCOOP,
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
                if (timer.seconds() > 1.0) { // Default time for simple auto actions
                    stopAll();
                }
                break;

            case INTAKING:
                if (timer.seconds() > 2.0) { // Run intake for a short burst
                    robot.intake.stop();
                    currentState = ActionState.IDLE;
                }
                break;

            case TRANSFER_PURPLE_TO_SCOOP:
                // After the full duration, return the transfer mechanism.
                if (timer.seconds() > 1.2) {
                    robot.transfer.LeftTransferReturn();
                    currentState = ActionState.IDLE;
                }
                break;

            case TRANSFER_GREEN_TO_SCOOP:
                // After the full duration, return the transfer mechanism.
                if (timer.seconds() > 1.2) {
                    robot.transfer.RightTransferReturn();
                    currentState = ActionState.IDLE;
                }
                break;

            case LAUNCH_FROM_SCOOP:
                // First, move the scoop up.
                if (!scoopHasBeenMoved) {
                    robot.scoop.ballUp();
                    scoopHasBeenMoved = true;
                    timer.reset();
                }
                // After a delay, bring it back down.
                if (timer.seconds() > 0.67) {
                    robot.scoop.ballDown();
                    currentState = ActionState.IDLE;
                }
                break;
        }
    }

    // ----- PUBLIC METHODS FOR TELEOP AND AUTO -----

    public void startTransferGreen() {
        if (isBusy()) return;
        currentState = ActionState.TRANSFER_GREEN_TO_SCOOP;
        robot.transfer.runRight();
        timer.reset();
    }

    public void startTransferPurple() {
        if (isBusy()) return;
        currentState = ActionState.TRANSFER_PURPLE_TO_SCOOP;
        robot.transfer.runLeft();
        timer.reset();
    }

    public void launchFromScoop() {
        if (isBusy()) return;
        scoopHasBeenMoved = false;
        currentState = ActionState.LAUNCH_FROM_SCOOP;
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
        // This method might need to be re-evaluated with the new action sequence.
        // For now, it will just transfer a purple ball.
        startTransferPurple();
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
