package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ActionManager is the definitive, unified class for controlling all robot mechanisms.
 * It is built as a state machine to handle complex, multi-step sequences like launching and intaking.
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Timed Actions: The durations for each step in the state machine (e.g., how long
 *    to run the transfer belt) are critical. These times in the `update()` method
 *    must be tuned to match the physical robot's performance to ensure reliability.
 * ---------------------------------------------------------------------------------
 */
public class ActionManager {

    private final RobotHardwareContainer robot;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean scoopHasBeenMoved = false;

    public enum ActionState {
        IDLE,
        INTAKING,
        TRANSFER_GREEN_ARTIFACT_TO_SCOOP,
        TRANSFER_PURPLE_ARTIFACT_TO_SCOOP,
        LAUNCH_FROM_SCOOP,
        WAITING_FOR_AUTO_ACTION
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robot) {
        this.robot = robot;
    }

    public void update() {
        switch (currentState) {
            case IDLE:
                break;

            case WAITING_FOR_AUTO_ACTION:
                if (timer.seconds() > 1.0) {
                    stopAll();
                }
                break;

            case INTAKING:
                // TODO: Tune the duration for the intake burst.
                if (timer.seconds() > 2.0) {
                    robot.intake.stop();
                    currentState = ActionState.IDLE;
                }
                break;

            case TRANSFER_PURPLE_ARTIFACT_TO_SCOOP:
                // TODO: Tune the duration for the purple artifact transfer.
                if (timer.seconds() > 1.2) {
                    robot.transfer.LeftTransferReturn();
                    currentState = ActionState.IDLE;
                }
                break;

            case TRANSFER_GREEN_ARTIFACT_TO_SCOOP:
                // TODO: Tune the duration for the green artifact transfer.
                if (timer.seconds() > 1.2) {
                    robot.transfer.RightTransferReturn();
                    currentState = ActionState.IDLE;
                }
                break;

            case LAUNCH_FROM_SCOOP:
                if (!scoopHasBeenMoved) {
                    robot.scoop.up();
                    scoopHasBeenMoved = true;
                    timer.reset();
                }
                // TODO: Tune the duration for the scoop launch sequence.
                if (timer.seconds() > 0.67) {
                    robot.scoop.down();
                    currentState = ActionState.IDLE;
                }
                break;
        }
    }

    // ----- PUBLIC METHODS FOR TELEOP AND AUTO -----

    public void startTransferGreenArtifact() {
        if (isBusy()) return;
        currentState = ActionState.TRANSFER_GREEN_ARTIFACT_TO_SCOOP;
        robot.transfer.runRight();
        timer.reset();
    }

    public void startTransferPurpleArtifact() {
        if (isBusy()) return;
        currentState = ActionState.TRANSFER_PURPLE_ARTIFACT_TO_SCOOP;
        robot.transfer.runLeft();
        timer.reset();
    }

    public void launchFromScoop() {
        if (isBusy()) return;
        scoopHasBeenMoved = false;
        currentState = ActionState.LAUNCH_FROM_SCOOP;
        timer.reset();
    }

    public void startIntake() {
        if(isBusy()) return;
        currentState = ActionState.WAITING_FOR_AUTO_ACTION;
        robot.intake.run();
        timer.reset();
    }

    /**
     * A specific, controlled action to clear Artifact jams.
     */
    public void clearJam() {
        robot.intake.reverse();
        robot.diverter.setPosition(DiverterHardware.GatePosition.NEUTRAL);
    }

    public void stopAll() {
        robot.intake.stop();
        robot.launcher.stop();
        robot.transfer.LeftTransferReturn();
        robot.transfer.RightTransferReturn();
        robot.scoop.down();
        currentState = ActionState.IDLE;
    }

    public boolean isBusy() {
        return currentState != ActionState.IDLE;
    }
}
