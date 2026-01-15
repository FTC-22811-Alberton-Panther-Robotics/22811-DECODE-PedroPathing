package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.ColorDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.intakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;

public class ActionManager {

    private final RobotHardwareContainer robot;
    private final intakeHardware intake;
    private final LauncherHardware launcher;
    private final TransferHardware transfer;
    // The ColorDiverterHardware is nullable, as it may not exist on all robot configurations.
    private final ColorDiverterHardware colorDiverter;
    private final ElapsedTime actionTimer = new ElapsedTime();

    public enum ActionState {
        IDLE,
        INTAKING,
        LAUNCHING_SPINUP,
        LAUNCHING_FIRE,
        REVERSING,
        ACTION_COMPLETE,

    }
    private ActionState currentState = ActionState.IDLE;

    public ActionManager(RobotHardwareContainer robotContainer) {
        this.robot = robotContainer;
        this.intake = robot.intake;
        this.launcher = robot.launcher;
        this.transfer = robot.transfer;
        this.colorDiverter = robot.colorDiverter; // This can be null
    }

    public void update() {
        switch (currentState) {
            case IDLE: case ACTION_COMPLETE: case REVERSING:
                break; // No timed logic in these states

            case INTAKING:
                if (actionTimer.seconds() > 2.5) { // Tune this time
                    stopAll();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;

            case LAUNCHING_SPINUP:
                if (launcher.isAtTargetSpeed()) {
                    actionTimer.reset();
                    currentState = ActionState.LAUNCHING_FIRE;
                }
                break;

            case LAUNCHING_FIRE:
                if (actionTimer.seconds() > 2.0) { // Tune this time
                    stopAll();
                    currentState = ActionState.ACTION_COMPLETE;
                }
                break;
        }
    }

    // ----- Public Methods to Trigger Actions -----

    public void startIntake() {
        if (isBusy()) return;
        currentState = ActionState.INTAKING;
        intake.run();
        actionTimer.reset();
    }

    public void startLaunch() {
        if (isBusy()) return;
        currentState = ActionState.LAUNCHING_SPINUP;
        launcher.spinUp();
    }

    public void reverseAll() {
        currentState = ActionState.REVERSING;
        intake.reverse();
        launcher.reverse();
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
