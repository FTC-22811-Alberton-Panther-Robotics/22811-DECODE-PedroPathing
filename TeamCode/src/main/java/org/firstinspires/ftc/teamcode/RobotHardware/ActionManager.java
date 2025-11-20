package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;

public class ActionManager {

    private final IntakeHardware intake;
    private final LauncherHardware launcher;
    private final TransferHardware transfer;
    private final ElapsedTime actionTimer = new ElapsedTime();

    public enum ActionState {
        IDLE,
        INTAKING,         // Running intake motor to collect artifacts
        LAUNCHING_SPINUP, // Spinning up flywheels
        LAUNCHING_FIRE,   // Pushing artifacts into flywheels
        REVERSING,
        ACTION_COMPLETE
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robotContainer) {
        this.intake = robotContainer.intake;
        this.launcher = robotContainer.launcher;
        this.transfer = robotContainer.transfer;
    }

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE: case REVERSING:
                break; // No timed logic in these states

            case INTAKING:
                // Run intake for a set duration to collect 3 artifacts
                if (actionTimer.seconds() > 2.5) { // Tune this time
                    stopAll();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;

            case LAUNCHING_SPINUP:
                // Wait for launcher to be at target speed
                if (launcher.isAtTargetSpeed()) {
                    transfer.run(); // Start transfer to fire
                    actionTimer.reset();
                    currentState = ActionState.LAUNCHING_FIRE;
                }
                break;

            case LAUNCHING_FIRE:
                // Run transfer for long enough to launch 3 artifacts
                if (actionTimer.seconds() > 2.0) { // Tune this time
                    stopAll();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;
        }
    }

    // ----- Public Methods to Trigger Actions -----

    /** Runs the intake for a duration to collect artifacts. */
    public void startIntake() {
        if (isBusy()) return;
        currentState = ActionState.INTAKING;
        intake.run();
        actionTimer.reset();
    }

    /** Starts the full launch sequence (spin-up, then fire). */
    public void startLaunch() {
        if (isBusy()) return;
        currentState = ActionState.LAUNCHING_SPINUP;
        launcher.spinUp();
    }

    /** Reverses all mechanisms to clear jams. */
    public void reverseAll() {
        currentState = ActionState.REVERSING;
        intake.reverse();
        launcher.reverse();
        transfer.reverse();
    }

    public boolean isBusy() {
        return currentState != ActionState.IDLE && currentState != ActionState.ACTION_COMPLETE;
    }

    public void completeAction() {
        currentState = ActionState.IDLE;
    }

    public void stopAll() {
        intake.stop();
        launcher.stop();
        transfer.stop();
        currentState = ActionState.IDLE;
    }
}
