package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;

/**
 * A helper class to manage advanced TeleOp driving features like field-centric
 * drive and target-locking headings. This class acts as an intelligent layer between
 * the driver's joystick inputs and the Pedro Pathing Follower, which handles the
 * underlying drive calculations.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Drive Modes:
 *    - ROBOT_CENTRIC: Standard, intuitive control. Forward on the joystick is always
 *      the front of the robot.
 *    - FIELD_CENTRIC: Forward on the joystick is always "downfield," away from the
 *      driver station, regardless of the robot's orientation.
 *    - TARGET_LOCK: A specialized field-centric mode where the robot will always try
 *      to turn and face the alliance goal. The driver's turning input is ignored.
 *
 * 2. Heading Kp: The `HEADING_KP` constant is a proportional gain for the Target Lock
 *    mode. It determines how aggressively the robot turns to stay locked onto the
 *    goal. If the robot is sluggish, increase this value. If it overshoots and
 *    oscillates, decrease this value.
 * ---------------------------------------------------------------------------------
 */
public class DriverAssist {

    private final Follower follower;
    private final CombinedLocalizer localizer;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        FC_TARGET_LOCK,
        RC_TARGET_LOCK
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

    // Proportional gain for the Target Lock mode.
    // Tune this for responsive but stable target locking.
    private static final double HEADING_KP = 0.3;
    private static final double TURNING_MULTIPLIER = 0.5; // limit turning power for more control

    public DriverAssist(Follower follower, CombinedLocalizer localizer) {
        this.follower = follower;
        this.localizer = localizer;
    }

    public void setMode(DriveMode mode) {
        this.currentMode = mode;
    }

    public DriveMode getMode() {
        return currentMode;
    }

    /**
     * Main update loop. Passes joystick inputs to the Follower and lets it handle
     * the driving calculations based on the selected mode.
     */
    public void update(double joyY, double joyX, double joyTurn) {
        Pose robotPose = follower.getPose();
        double headingError = 0;
        double calculatedTurn = 0;
        joyTurn = squareInputWithSign(joyTurn) * TURNING_MULTIPLIER; // Limit turning power for more control
        joyY = squareInputWithSign(joyY);
        joyX = squareInputWithSign(joyX);

        // If localization is not stable, do not send drive commands.
        if (robotPose == null) {
            currentMode = DriveMode.ROBOT_CENTRIC;
        } else {
            headingError = getSignedAngleDifference(robotPose.getHeading(), calculateHeadingToGoal(robotPose));
            calculatedTurn = HEADING_KP * headingError;
            calculatedTurn = Math.max(-1.0, Math.min(1.0, calculatedTurn));
        }

        switch (currentMode) {
            case ROBOT_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, true);
                break;

            case FIELD_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, false);
                break;

            case FC_TARGET_LOCK:
                follower.setTeleOpDrive(joyY, joyX, calculatedTurn, false);
                break;

            case RC_TARGET_LOCK:
                follower.setTeleOpDrive(joyY, joyX, calculatedTurn, true);
                break;
        }
    }

    /**
     * Calculates the smallest signed angle difference between a current and target angle.
     * The result will be in the range [-PI, PI], which is crucial for proportional control.
     * A positive result means a counter-clockwise turn is needed.
     * A negative result means a clockwise turn is needed.
     *
     * @param currentAngle The current angle in radians.
     * @param targetAngle The target angle in radians.
     * @return The signed angle difference in radians.
     */
    private double getSignedAngleDifference(double currentAngle, double targetAngle) {
        double diff = targetAngle - currentAngle;
        // Normalize the difference to be within the range [-PI, PI]
        while (diff > Math.PI) {
            diff -= 2 * Math.PI;
        }
        while (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }
        return diff;
    }

    /**
     * Calculates the absolute field heading (in radians) from the robot's current position to the goal.
     */
    public double calculateHeadingToGoal(Pose robotPose)
    {
        Pose targetGoal = (GameState.alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        return Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );
    }

    /**
     * Takes gamepad joystick input which will always be in the [-1, 1] range, and squares it, while
     * keeping the sign. Squaring a decimal number (between 0 and 1, positive or negative) makes it
     * smaller. In this way we can make small joystick adjustments result in even smaller robot
     * movements to give fine control when attempting to move precisely while keeping fast movements
     * fast.
     * @param input
     * @return
     */
    public double squareInputWithSign(double input) {
        return Math.copySign(input * input, input);
    }

}
