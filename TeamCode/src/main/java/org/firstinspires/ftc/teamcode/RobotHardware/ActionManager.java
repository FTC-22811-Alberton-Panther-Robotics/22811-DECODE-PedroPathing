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
    private static final double LAUNCHER_SPINUP_TIMEOUT_SECONDS = 1.5;
    private static final double SCOOP_CYCLE_SECONDS = 0.67;
    // TODO: Tune these new constants for the autonomous scoring sequence
    private static final double AUTO_SCORE_INTAKE_TIME = 0.25; // Time to run intake to seat artifact
    private static final double AUTO_SCORE_TRANSFER_TIME = 1.2;  // Time to run the transfer kicker

    public enum ActionState {
        IDLE,
        INTAKING,
        TRANSFER_GREEN_ARTIFACT_TO_SCOOP,
        TRANSFER_PURPLE_ARTIFACT_TO_SCOOP,
        // Unified Launch Sequence
        LAUNCHER_SPINUP,
        SCOOP_CYCLE,
        WAITING_FOR_AUTO_ACTION,

        // New states for the complete autonomous scoring sequence
        AUTO_SCORE_INTAKE,
        AUTO_SCORE_TRANSFER,
        AUTO_SCORE_LAUNCH_SPINUP,
        AUTO_SCORE_SCOOP_CYCLE
    }
    private ActionState currentState = ActionState.IDLE;
    private char autoScoreArtifactColor;


    public ActionManager(RobotHardwareContainer robot) {
        this.robot = robot;
    }

    public void update() {
        switch (currentState) {
            case IDLE:
            case INTAKING:
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

            // --- Autonomous Scoring Sequence ---
            case AUTO_SCORE_INTAKE:
                // Step 1: Run intake briefly to seat the artifact
                if (timer.seconds() > AUTO_SCORE_INTAKE_TIME) {
                    robot.intake.stop();
                    // Move to Step 2: Run the correct transfer kicker
                    currentState = ActionState.AUTO_SCORE_TRANSFER;
                    if (autoScoreArtifactColor == 'P') {
                        robot.transfer.runLeft();
                    } else {
                        robot.transfer.runRight();
                    }
                    timer.reset();
                }
                break;

            case AUTO_SCORE_TRANSFER:
                // Step 2: Wait for the transfer to complete
                if (timer.seconds() > AUTO_SCORE_TRANSFER_TIME) {
                    if (autoScoreArtifactColor == 'P') {
                        robot.transfer.returnLeft();
                    } else {
                        robot.transfer.returnRight();
                    }
                    // Move to Step 3: Launch the artifact
                    currentState = ActionState.AUTO_SCORE_LAUNCH_SPINUP;
                    if (!robot.launcher.isLauncherOn()) {
                        robot.launcher.toggleLauncher();
                    }
                    timer.reset();
                }
                break;

            case AUTO_SCORE_LAUNCH_SPINUP:
                 // Step 3a: Wait for the launcher to get to speed
                if (robot.launcher.isAtTargetSpeed() || timer.seconds() > LAUNCHER_SPINUP_TIMEOUT_SECONDS) {
                    currentState = ActionState.AUTO_SCORE_SCOOP_CYCLE;
                    robot.scoop.up();
                    timer.reset();
                }
                break;

            case AUTO_SCORE_SCOOP_CYCLE:
                // Step 3b: Wait for the scoop cycle to finish
                if (timer.seconds() > SCOOP_CYCLE_SECONDS) {
                    robot.scoop.down();
                    currentState = ActionState.IDLE; // Sequence is complete
                }
                break;
            
            // This is the old, simpler launch sequence. It can still be used if needed.
            case LAUNCHER_SPINUP:
                // Wait for the launcher to be at speed, or for the timeout to expire.
                if (robot.launcher.isAtTargetSpeed() || timer.seconds() > LAUNCHER_SPINUP_TIMEOUT_SECONDS) {
                    currentState = ActionState.SCOOP_CYCLE;
                    robot.scoop.up();
                    timer.reset();
                }
                break;
            case SCOOP_CYCLE:
                // Wait for the scoop to be up for the specified duration.
                if (timer.seconds() > SCOOP_CYCLE_SECONDS) {
                    robot.scoop.down();
                    currentState = ActionState.IDLE; // Correctly finish the action
                }
                break;
        }
    }

    // ----- PUBLIC METHODS -----
    
    /**
     * Initiates the complete autonomous scoring sequence: seats, transfers, and launches.
     * @param artifactColor The color of the artifact ('P' or 'G') to determine which transfer to run.
     */
    public void autonomousScore(char artifactColor) {
        if (isBusy()) return;
        this.autoScoreArtifactColor = artifactColor;
        // Start Step 1: Run the intake
        currentState = ActionState.AUTO_SCORE_INTAKE;
        robot.intake.run();
        timer.reset();
    }
    
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
     * Initiates the unified launch sequence. Can be called from Auto or TeleOp.
     * It will turn on the launcher if it's off, wait for it to reach speed, then fire.
     */
    public void launch() {
        if (isBusy()) return;

        // If launcher isn't on, turn it on.
        if (!robot.launcher.isLauncherOn()) {
            robot.launcher.toggleLauncher();
        }
        currentState = ActionState.LAUNCHER_SPINUP;
        timer.reset();
    }

    public void startIntake() {
        if(isBusy()) return;
        currentState = ActionState.INTAKING;
        robot.intake.run();
    }
    
    public void stopIntake(){
        if(currentState == ActionState.INTAKING){
            robot.intake.stop();
            currentState = ActionState.IDLE;
        }
    }

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
        return currentState != ActionState.IDLE;
    }

    public ActionState getCurrentState() {
        return currentState;
    }
}
