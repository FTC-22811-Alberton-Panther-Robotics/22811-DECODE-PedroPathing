package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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
 * 3. (CRITICAL) Distance-to-Velocity Mapping: The automatic speed control uses a 3-point
 *    quadratic interpolation. To tune:
 *      a. Place the robot at your desired close, middle, and far shot positions.
 *      b. Record the poses in `CLOSE_SHOT_POSE`, `MID_SHOT_POSE`, and `FAR_SHOT_POSE`.
 *      c. Manually find the flywheel RPM needed for a successful shot at each of these three distances.
 *      d. Set these values in `CLOSE_SHOT_RPM`, `MID_SHOT_RPM`, and `FAR_SHOT_RPM`.
 *      e. The code will automatically calculate the speed for any distance in between.
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

    // --- Launcher Speed Interpolation Constants (3-Point Quadratic) ---
    private static final Pose CLOSE_SHOT_POSE = new Pose(108, 108, 45);
    private static final double CLOSE_SHOT_RPM = 2700;
    private static final Pose MID_SHOT_POSE = new Pose(83.53, 83.53, 45); // New mid-point for the curve
    private static final double MID_SHOT_RPM = 3000;
    private static final Pose FAR_SHOT_POSE = new Pose(84, 11, 65);
    private static final double FAR_SHOT_RPM = 4000;

    // Cached values for interpolation, calculated once on first use.
    private static double closeShotDistance = -1, midShotDistance = -1, farShotDistance = -1;
    private static double l0_denom = 0, l1_denom = 0, l2_denom = 0; // Denominators for Lagrange interpolation

    // --- Physical & State Constants ---
    public static final double RPM_TOLERANCE = 10;
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
     * Calculates the required launch speed using 3-point quadratic Lagrange interpolation.
     */
    private double interpolate(double currentDistance) {
        // On the first run, calculate the distances and the constant denominators for the formula.
        if (closeShotDistance < 0) {
            Pose goal = FieldPosePresets.RED_GOAL_TARGET;
            closeShotDistance = Math.hypot(goal.getX() - CLOSE_SHOT_POSE.getX(), goal.getY() - CLOSE_SHOT_POSE.getY());
            midShotDistance = Math.hypot(goal.getX() - MID_SHOT_POSE.getX(), goal.getY() - MID_SHOT_POSE.getY());
            farShotDistance = Math.hypot(goal.getX() - FAR_SHOT_POSE.getX(), goal.getY() - FAR_SHOT_POSE.getY());

            // Pre-calculate the denominators for the Lagrange basis polynomials to save cycles.
            l0_denom = (closeShotDistance - midShotDistance) * (closeShotDistance - farShotDistance);
            l1_denom = (midShotDistance - closeShotDistance) * (midShotDistance - farShotDistance);
            l2_denom = (farShotDistance - closeShotDistance) * (farShotDistance - midShotDistance);
        }

        // Calculate the numerators for the Lagrange basis polynomials with the current distance.
        double l0_num = (currentDistance - midShotDistance) * (currentDistance - farShotDistance);
        double l1_num = (currentDistance - closeShotDistance) * (currentDistance - farShotDistance);
        double l2_num = (currentDistance - closeShotDistance) * (currentDistance - midShotDistance);

        // Combine to get the final interpolated RPM.
        double calculatedSpeed = (CLOSE_SHOT_RPM * l0_num / l0_denom) +
                                 (MID_SHOT_RPM * l1_num / l1_denom) +
                                 (FAR_SHOT_RPM * l2_num / l2_denom);

        return Math.max(0, calculatedSpeed);
    }

    /** Sets the flywheel velocity in RPM. */
    public void setLaunchSpeed(double rpm) {
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
        if (!isSpinning || targetRPM == 0) return false;
        double leftError = Math.abs(targetRPM - getLeftFlywheelRPM());
        double rightError = Math.abs(targetRPM - getRightFlywheelRPM());
        return (leftError < RPM_TOLERANCE) && (rightError < RPM_TOLERANCE);
    }

    // --- Getter Methods ---
    public boolean isLauncherOn() { return isSpinning; }
    public double getLeftFlywheelRPM() { return (leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV); }
    public double getRightFlywheelRPM() { return (rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV); }
    public double getTargetRPM() { return isSpinning ? targetRPM : 0; }
    public double getLeftFlywheelCurrent() { return leftFlywheel.getCurrent(CurrentUnit.AMPS); }
    public double getRightFlywheelCurrent() { return rightFlywheel.getCurrent(CurrentUnit.AMPS); }
}
