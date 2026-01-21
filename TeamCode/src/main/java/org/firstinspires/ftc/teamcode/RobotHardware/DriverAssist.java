package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

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

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET_LOCK
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

    // TODO: Tune this Kp value for responsive but stable target locking.
    private static final double HEADING_KP = 0.8;

    public DriverAssist(Follower follower) {
        this.follower = follower;
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
    public void update(double joyY, double joyX, double joyTurn, GameState.Alliance alliance) {
        Pose robotPose = follower.getPose();

        // DEFINITIVE FIX: If the pose is null, do not send ANY drive commands.
        // Immediately return and wait for the next cycle for localization to stabilize.
        if (robotPose == null) {
            return;
        }

        switch (currentMode) {
            case ROBOT_CENTRIC:
                // The `true` here specifies that the joystick inputs are robot-centric.
                follower.setTeleOpDrive(joyY, joyX, joyTurn, true);
                break;

            case FIELD_CENTRIC:
                // The `false` here specifies that the inputs are for field-centric mode.
                follower.setTeleOpDrive(joyY, joyX, joyTurn, false);
                break;

            case TARGET_LOCK:
                double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(alliance, robotPose), robotPose.getHeading());
                double calculatedTurn = HEADING_KP * headingError;
                calculatedTurn = Math.max(-1.0, Math.min(1.0, calculatedTurn));

                // Let the Follower handle field-centric joysticks, but override the turn value with our own.
                follower.setTeleOpDrive(joyY, joyX, calculatedTurn, false);
                break;
        }
    }
    
    /**
     * Calculates the absolute field heading (in radians) from the robot's current position to the goal.
     * @param robotPose The robot's current pose, guaranteed not to be null by the main update method.
     */
    public double calculateHeadingToGoal(GameState.Alliance alliance, Pose robotPose)
    {
        Pose targetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;
        
        return Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );
    }


    /**
     * Resets the robot's heading in the localization system. This is useful if the robot
     * gets bumped or its orientation becomes inaccurate.
     */
    public void resetHeading() {
        Pose currentPose = follower.getPose();
        if (currentPose != null) {
            // If we have a pose, just reset the heading to 90 degrees (facing forward).
            follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));
        } else {
            // If the pose is null, reset to a default pose (0, 0) with a 90-degree heading.
            follower.setPose(new Pose(0, 0, Math.toRadians(90)));
        }
    }

}
