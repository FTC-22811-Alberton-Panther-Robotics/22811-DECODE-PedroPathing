package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ActionManager is the definitive, unified class for controlling all robot mechanisms.
 * It is built as a state machine to handle complex, multi-step sequences like shooting and intaking.
 * This replaces the previous BallManager and the old, broken ActionManager.
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

            case SHOOT_TRANSFER_LEFT:
                if (timer.seconds() > 0.5) {
                    robot.ballTransfer.leftReturn();
                    currentState = ActionState.SHOOT_SCOOP;
                    timer.reset();
                }
                break;

            case SHOOT_TRANSFER_RIGHT:
                if (timer.seconds() > 0.5) {
                    robot.ballTransfer.rightReturn();
                    currentState = ActionState.SHOOT_SCOOP;
                    timer.reset();
                }
                break;

            case SHOOT_SCOOP:
                if (timer.seconds() > 0.67) {
                    robot.scooper.ballDown();
                    currentState = ActionState.IDLE;
                }
                break;
        }
    }

    // ----- PUBLIC METHODS FOR TELEOP AND AUTO -----

    public void startGreenBallShoot() {
        if (isBusy()) return;
        currentState = ActionState.SHOOT_TRANSFER_RIGHT;
        robot.launcher.spinUp();
        robot.ballTransfer.rightTransfer();
        robot.scooper.ballUp();
        timer.reset();
    }

    public void startPurpleBallShoot() {
        if (isBusy()) return;
        currentState = ActionState.SHOOT_TRANSFER_LEFT;
        robot.launcher.spinUp();
        robot.ballTransfer.leftTransfer();
        robot.scooper.ballUp();
        timer.reset();
    }

    public void setDiverterToGreen() {
        if (robot.ballDiverter != null) {
            robot.ballDiverter.greenBall();
        }
    }

    public void setDiverterToPurple() {
        if (robot.ballDiverter != null) {
            robot.ballDiverter.purpleBall();
        }
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
        robot.launcher.spinUp();
        timer.reset();
    }

    public void startSmartLaunch(GameState.ObeliskPattern pattern) {
        // This can be expanded with pattern-specific logic
        // For now, it defaults to a standard launch sequence.
        if (isBusy()) return;
        // This is just an example; you can call startPurpleBallShoot or startGreenBallShoot here.
        startPurpleBallShoot(); 
    }

    public void stopAll() {
        robot.intake.stop();
        robot.launcher.stop();
        robot.ballTransfer.leftReturn();
        robot.ballTransfer.rightReturn();
        robot.scooper.ballDown();
        currentState = ActionState.IDLE;
    }

    public boolean isBusy() {
        return currentState != ActionState.IDLE;
    }
}
