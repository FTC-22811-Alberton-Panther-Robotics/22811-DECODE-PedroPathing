package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

/**
 * A helper class to manage advanced TeleOp driving features like field-centric
 * drive and target-locking headings.
 */
public class DriverAssist {

    private final Follower follower;

    public enum DriveMode {
        ROBOT_CENTRIC, // Standard controls, relative to the robot's front.
        FIELD_CENTRIC, // Controls are relative to the field, not the robot.
        TARGET_LOCK    // Field-centric with automatic heading alignment to a target.
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

    // Proportional gain for the target-lock heading controller. This can be tuned.
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
     * Main update loop. Calculates and applies drive powers based on the current mode.
     * @param joyY The joystick's Y input (forward/backward).
     * @param joyX The joystick's X input (strafe left/right).
     * @param joyTurn The joystick's turn input.
     * @param alliance The current alliance, used for selecting the correct goal in TARGET_LOCK mode.
     */
    public void update(double joyY, double joyX, double joyTurn, GameState.Alliance alliance) {
        double forward, strafe, turn;

        Pose robotPose = follower.getPose();
        double robotHeading = robotPose.getHeading();

        switch (currentMode) {
            case ROBOT_CENTRIC:
                // In robot-centric mode, the joystick inputs are applied directly.
                // Forward on the stick is always forward for the robot.
                forward = joyY;
                strafe = joyX;
                turn = joyTurn;
                break;

            case TARGET_LOCK:
                // In target-lock mode, translation is field-centric.
                double sin_tl = Math.sin(-robotHeading);
                double cos_tl = Math.cos(-robotHeading);
                forward = joyY * cos_tl - joyX * sin_tl;
                strafe = joyY * sin_tl + joyX * cos_tl;

                // But the robot's rotation is automatically handled by a P-controller to point at the goal.
                double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(alliance), robotHeading);
                turn = HEADING_KP * headingError;
                turn = Math.max(-1.0, Math.min(1.0, turn)); // Clamp to valid power range.
                break;

            case FIELD_CENTRIC:
            default:
                // In field-centric mode, we rotate the joystick inputs by the inverse of the robot's heading.
                // This makes "forward" on the stick always drive away from the driver, down the field.
                double sin_fc = Math.sin(-robotHeading);
                double cos_fc = Math.cos(-robotHeading);
                forward = joyY * cos_fc - joyX * sin_fc;
                strafe = joyY * sin_fc + joyX * cos_fc;
                turn = joyTurn; // Turn is still controlled manually.
                break;
        }

        // Send the final calculated powers to the follower/drivetrain.
        follower.setTeleOpDrive(forward, strafe, turn);
    }
    
    /**
     * Calculates the absolute field heading (in radians) from the robot's current position
     * to the center of the correct alliance goal.
     * @param alliance The current alliance, to select the correct goal.
     * @return The field-relative heading in radians.
     */
    public double calculateHeadingToGoal(GameState.Alliance alliance)
    {
        Pose robotPose = follower.getPose();
        
        // Select the correct goal from our presets.
        Pose targetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;
        
        // Use Math.atan2 to calculate the angle between the robot's position and the goal's position.
        return Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );
    }
}
