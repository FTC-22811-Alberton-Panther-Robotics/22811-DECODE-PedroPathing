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

    // --- Tuning Constants ---
    private static final double LAUNCHER_SPINUP_TIMEOUT_SECONDS = 1.5; // Max time to wait for the launcher to get to speed.
    private static final double SCOOP_CYCLE_SECONDS = 0.67; // How long the scoop stays up to fire.

    public enum ActionState {
        IDLE,
        INTAKING,
        TRANSFER_GREEN_ARTIFACT_TO_SCOOP,
        TRANSFER_PURPLE_ARTIFACT_TO_SCOOP,
        AUTO_LAUNCHER_SPINUP,       // Autonomous: Spin up launcher and wait for it to be ready.
        AUTO_SCOOP_CYCLE,           // Autonomous: Fire by moving the scoop.
        TELEOP_SCOOP_CYCLE,         // TeleOp: Manually fire the scoop (assumes launcher is already on).
        WAITING_FOR_AUTO_ACTION
    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robot) {
        this.robot = robot;
    }

    public void update() {
        switch (currentState) {
            case IDLE:
            case INTAKING: // Manually started/stopped
                break;

            case WAITING_FOR_AUTO_ACTION:
                if (timer.seconds() > 1.0) {
                    stopAll();
                }
                break;

            case TRANSFER_PURPLE_ARTIFACT_TO_SCOOP:
                // TODO: Tune the duration for the purple artifact transfer.
                if (timer.seconds() > 1.2) {
                    robot.transfer.returnLeft();
                    currentState = ActionState.IDLE;
                }
                break;

            case TRANSFER_GREEN_ARTIFACT_TO_SCOOP:
                // TODO: Tune the duration for the green artifact transfer.
                if (timer.seconds() > 1.2) {
                    robot.transfer.returnRight();
                    currentState = ActionState.IDLE;
                }
                break;

            // --- Autonomous Launch Sequence ---
            case AUTO_LAUNCHER_SPINUP:
                // In this state, we check if the launcher is at its target speed.
                if (robot.launcher.isAtTargetSpeed()) {
                    // It's ready! Move to the next state to fire.
                    currentState = ActionState.AUTO_SCOOP_CYCLE;
                    robot.scoop.up();
                    timer.reset();
                } else if (timer.seconds() > LAUNCHER_SPINUP_TIMEOUT_SECONDS) {
                    // It took too long to spin up. Abort the launch and stop the launcher to save battery.
                    stopAll();
                }
                break;

            case AUTO_SCOOP_CYCLE:
                // The scoop is up. We wait for the cycle time to complete.
                if (timer.seconds() > SCOOP_CYCLE_SECONDS) {
                    robot.scoop.down();
                    // Auto sequence is over, everything should stop.
                    stopAll();
                }
                break;

            // --- TeleOp Manual Firing ---
            case TELEOP_SCOOP_CYCLE:
                // The scoop is up. We wait for the cycle time to complete.
                if (timer.seconds() > SCOOP_CYCLE_SECONDS) {
                    robot.scoop.down();
                    // Return to IDLE, leaving the launcher on for the next shot.
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

    /**
     * Initiates the fully autonomous launch sequence.
     * This will start the launcher, wait for it to reach speed, and then fire.
     */
    public void launchFromScoopAutonomous() {
        // Do not start a new launch if another sequence is running.
        if (isBusy()) return;

        // Start the flywheel motor.
        robot.launcher.start();

        // Transition to the state that waits for the flywheel to get to speed.
        currentState = ActionState.AUTO_LAUNCHER_SPINUP;

        // Reset the timer to begin the spin-up timeout.
        timer.reset();
    }

    /**
     * Initiates the manual TeleOp launch sequence.
     * This ONLY cycles the scoop. It assumes the driver has already turned the launcher on.
     */
    public void launchFromScoopTeleop() {
        // Allow this to be spammed to re-try a shot, but don't interrupt other critical actions.
        if (isBusy() && currentState != ActionState.TELEOP_SCOOP_CYCLE) return;

        // This action does not start the launcher. That's the driver's job.
        currentState = ActionState.TELEOP_SCOOP_CYCLE;
        robot.scoop.up();
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
        robot.transfer.returnLeft();
        robot.transfer.returnRight();
        robot.scoop.down();
        currentState = ActionState.IDLE;
    }

    public boolean isBusy() {
        // Return true if the state machine is in any state other than IDLE.
        return currentState != ActionState.IDLE;
    }

    public ActionState getCurrentState() {
        return currentState;
    }
}
