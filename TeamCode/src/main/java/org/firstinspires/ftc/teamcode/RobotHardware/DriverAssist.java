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
        TARGET_LOCK
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

    // TODO: Tune this Kp value for responsive but stable target locking.
    private static final double HEADING_KP = 0.8;

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
        //Pose robotPose = localizer.getPose();

//        // This is a comprehensive guard clause to prevent crashes on the first loop.
//        // It checks for null objects, our reliability flag, the hardware's NaN flag,
//        // and a manual check of the pose contents to prevent passing invalid data.
//        if (robotPose == null || !localizer.isPoseReliable() || localizer.isNAN()
//                || Double.isNaN(robotPose.getX()) || Double.isNaN(robotPose.getY()) || Double.isNaN(robotPose.getHeading())) {
//            follower.setTeleOpDrive(0,0,0,true); // Send a zero power command to be safe
//            return;
//        }

        switch (currentMode) {
            case ROBOT_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, true);
                break;

            case FIELD_CENTRIC:
                follower.setTeleOpDrive(joyY, joyX, joyTurn, false);
                break;

            case TARGET_LOCK:
                //double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(robotPose), robotPose.getHeading());
                //double calculatedTurn = HEADING_KP * headingError;
                //calculatedTurn = Math.max(-1.0, Math.min(1.0, calculatedTurn));

                //follower.setTeleOpDrive(joyY, joyX, calculatedTurn, false);
                follower.setTeleOpDrive(joyY, joyX, joyTurn, false);
                break;
        }
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
}
