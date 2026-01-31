package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Manages the robot's launcher, which consists of two flywheel motors.
 * This class includes logic for automatically adjusting the flywheel speed based on the
 * robot's distance to the goal.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Motor Directions: The two flywheel motors must spin in opposite directions to grip
 *    and launch the Artifact. Verify this in the `init()` method.
 *
 * 2. (CRITICAL) PIDF Tuning for Velocity: The `LAUNCHER_PIDF` coefficients are ESSENTIAL
 *    for the flywheels to maintain a constant, accurate velocity. Without good PIDF
 *    tuning, shots will be inconsistent. Use a dedicated tuning OpMode to find these values.
 *
 * 3. (CRITICAL) Distance-to-Velocity Mapping: The automatic speed control uses a linear mapping
 *    between two reference points. To tune:
 *      a. Place the robot at your desired close-range shot position and record the pose in `CLOSE_SHOT_POSE`.
 *      b. Manually find the flywheel RPM needed for a successful shot (`CLOSE_SHOT_RPM`).
 *      c. Repeat for a far-range shot with `FAR_SHOT_POSE` and `FAR_SHOT_RPM`.
 *      d. The code will automatically calculate the speed for any distance in between.
 *
 * 4. Ticks Per Revolution: The `TICKS_PER_REV` value must match your motor's specifications.
 *    The default is 28 for a goBILDA 5202 series motor.
 *
 * 5. RPM Tolerance: `RPM_TOLERANCE` defines how close the motors must be to the target
 *    speed for `isAtTargetSpeed()` to return true. Tune this for a balance of speed and accuracy.
 * ---------------------------------------------------------------------------------
 */
public class LauncherHardware {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private final Follower follower;

    // --- PIDF Tuning ---
    // TODO: Tune these PIDF coefficients for your launcher motors.
    public static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(300, 0, 0, 10);

    // --- Launcher Speed Interpolation Constants ---
    // TODO: Tune these two points to define the shot curve for your robot.
    private static final Pose CLOSE_SHOT_POSE = new Pose(75, 81, 0); // A sample close shot position
    private static final double CLOSE_SHOT_RPM = 1200.0;
    private static final Pose FAR_SHOT_POSE = new Pose(84, 11, 0);   // A sample far shot position
    private static final double FAR_SHOT_RPM = 1350.0;

    // Cached distances for interpolation, calculated once on first use.
    private static double closeShotDistance = -1;
    private static double farShotDistance = -1;

    // --- Physical & State Constants ---
    public static final double RPM_TOLERANCE = 25;
    public static final double TICKS_PER_REV = 28;
    private double targetRPM = 0;
    private boolean isSpinning = false; // The current state of the launcher

    public LauncherHardware(Follower follower) {
        this.follower = follower;
    }

    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        // TODO: Verify motor directions. They should spin in opposite directions.
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the tuned PIDF coefficients
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    /**
     * This is the main update loop. If the launcher is spinning, it calculates and sets
     * the required speed based on the robot's distance to the goal.
     */
    public void update(GameState.Alliance alliance) {
        if (!isSpinning) {
            // If the launcher is off, ensure motors are stopped.
            if (targetRPM != 0) stop();
            return;
        }

        Pose robotPose = follower.getPose();
        if (robotPose == null) {
            // Safety check: if localization is lost, default to a known safe speed.
            setLaunchSpeed(CLOSE_SHOT_RPM);
            return;
        }

        Pose targetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        double distance = Math.hypot(targetGoal.getX() - robotPose.getX(), targetGoal.getY() - robotPose.getY());
        double calculatedSpeed = interpolate(distance);
        setLaunchSpeed(calculatedSpeed);
    }

    /**
     * Calculates the required launch speed using linear interpolation between the two
     * defined shot reference points (CLOSE_SHOT and FAR_SHOT).
     */
    private double interpolate(double currentDistance) {
        // Calculate the reference distances on the first call to be safe.
        if (closeShotDistance < 0) {
            // The tuning points are on the red side, so we use the red goal as our reference for both.
            closeShotDistance = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - CLOSE_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - CLOSE_SHOT_POSE.getY());
            farShotDistance = Math.hypot(FieldPosePresets.RED_GOAL_TARGET.getX() - FAR_SHOT_POSE.getX(), FieldPosePresets.RED_GOAL_TARGET.getY() - FAR_SHOT_POSE.getY());
        }

        // Using the classic linear interpolation formula y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double calculatedSpeed = CLOSE_SHOT_RPM + ((currentDistance - closeShotDistance) * (FAR_SHOT_RPM - CLOSE_SHOT_RPM)) / (farShotDistance - closeShotDistance);
        return Math.max(0, calculatedSpeed); // Prevent negative speeds
    }

    /** Sets the flywheel velocity in RPM. */
    private void setLaunchSpeed(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    /** Starts spinning the flywheels. */
    public void start() {
        isSpinning = true;
    }

    /** Stops the flywheels and resets the target speed. */
    public void stop() {
        isSpinning = false;
        setLaunchSpeed(0);
    }

    /** Toggles the launcher flywheels on or off. */
    public void toggleLauncher() {
        isSpinning = !isSpinning;
    }

    /** Runs the motors in reverse to help clear jams. */
    public void reverse() {
        rightFlywheel.setVelocity(-1000);
        leftFlywheel.setVelocity(-1000);
    }

    /**
     * Checks if the flywheels are at their target speed within the defined tolerance.
     * This is crucial for ActionManager to decide when to fire.
     * @return true if both flywheels are at the target speed.
     */
    public boolean isAtTargetSpeed() {
        // Must be spinning and have a non-zero target to be "at speed"
        if (!isSpinning || targetRPM == 0) {
            return false;
        }
        double leftError = Math.abs(targetRPM - getLeftFlywheelRPM());
        double rightError = Math.abs(targetRPM - getRightFlywheelRPM());
        return (leftError < RPM_TOLERANCE) && (rightError < RPM_TOLERANCE);
    }

    // --- Getter Methods ---
    public boolean isLauncherOn() { return isSpinning; }
    public double getLeftFlywheelRPM() { return (leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV); }
    public double getRightFlywheelRPM() { return (rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV); }
    public double getTargetRPM() { return isSpinning ? targetRPM : 0; }
}
