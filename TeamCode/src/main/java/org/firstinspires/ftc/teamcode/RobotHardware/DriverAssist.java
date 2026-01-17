package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

/**
 * A helper class to manage advanced TeleOp driving features like field-centric
 * drive and target-locking headings. This has been refactored to work correctly
 * with the Pedro Pathing library's internal field-centric calculations.
 */
public class DriverAssist {

    private final Follower follower;

    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
        TARGET_LOCK
    }
    private DriveMode currentMode = DriveMode.ROBOT_CENTRIC;

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
}
