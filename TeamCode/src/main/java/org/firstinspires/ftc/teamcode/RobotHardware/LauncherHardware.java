package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Manages the robot's launcher, which consists of two flywheel motors.
 * This class includes logic for automatically adjusting the flywheel speed based on the
 * robot's distance to the goal, as well as manual overrides.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Motor Directions: The two flywheel motors must spin in opposite directions to grip
 *    and launch the Artifact. If they both spin the same way, reverse one of the motors
 *    in the `init()` method.
 *
 * 2. PIDF Tuning for Velocity: The most critical tuning for the launcher is the PIDF
 *    coefficients for the flywheel motors. These are set in the motor configuration or
 *    can be set in code. Correct PIDF values are ESSENTIAL for the flywheels to maintain
 *    a constant, accurate velocity when an Artifact is passing through, which is key
 *    to consistent launching.
 *
 * 3. Distance-to-Velocity Mapping: The automatic speed control uses a linear mapping
 *    between the robot's distance to the goal and the desired flywheel velocity. To tune:
 *      a. Set `MIN_SHOT_DISTANCE` to the closest distance you expect to launch from.
 *      b. Manually find the flywheel velocity (`MIN_SHOT_VELOCITY`) needed to make a
 *         shot from that minimum distance.
 *      c. Repeat for `MAX_SHOT_DISTANCE` and `MAX_SHOT_VELOCITY`.
 *      d. The code will then automatically interpolate for any distance in between.
 *
 * 4. Ticks Per Revolution: The `setFlywheelVelocity` method assumes a specific motor
 *    (goBILDA 5202 series with 28 ticks/rev). If you use a different motor, you MUST
 *    update the ticks per revolution value in that calculation for the RPM to be accurate.
 * ---------------------------------------------------------------------------------
 */
public class LauncherHardware {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private final Follower follower;
    private final TurretHardware turret;

    // --- Tunable Constants for Automatic Speed Control ---
    // TODO: Tune all four of these values to create an accurate distance-to-velocity mapping.
    public static double MIN_SHOT_DISTANCE = 36; // Inches
    public static double MAX_SHOT_DISTANCE = 120; // Inches
    public static double MIN_SHOT_VELOCITY = 2000; // RPM
    public static double MAX_SHOT_VELOCITY = 3500; // RPM

    // TODO: Tune this value for a reasonable manual adjustment increment.
    private static final double MANUAL_VELOCITY_INCREMENT = 100; // RPM per bumper press

    // --- State Variables ---
    private double manualVelocityOffset = 0.0;
    private double currentTargetVelocity = 0.0;
    private boolean isLauncherOn = false; // Default to off

    public LauncherHardware(Follower follower, TurretHardware turret) {
        this.follower = follower;
        this.turret = turret;
    }

    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        // TODO: Verify motor directions. They should spin in opposite directions.
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        stop();
    }

    /**
     * This is the main update loop for the launcher.
     * It calculates the required flywheel speed based on distance and applies it.
     */
    public void update() {
        if (!isLauncherOn) {
            stop();
            return;
        }

        Pose robotPose = follower.getPose();
        Pose targetGoal = turret.getTargetGoal();

        if (robotPose == null || targetGoal == null) {
            stop(); // Safety check
            return;
        }

        double distanceToGoal = Math.hypot(targetGoal.getX() - robotPose.getX(), targetGoal.getY() - robotPose.getY());

        // Linear interpolation to map distance to velocity
        double slope = (MAX_SHOT_VELOCITY - MIN_SHOT_VELOCITY) / (MAX_SHOT_DISTANCE - MIN_SHOT_DISTANCE);
        double calculatedVelocity = MIN_SHOT_VELOCITY + slope * (distanceToGoal - MIN_SHOT_DISTANCE);
        
        // Clamp the velocity to the defined min/max range
        calculatedVelocity = Math.max(MIN_SHOT_VELOCITY, Math.min(MAX_SHOT_VELOCITY, calculatedVelocity));

        this.currentTargetVelocity = calculatedVelocity + manualVelocityOffset;
        setFlywheelVelocity(this.currentTargetVelocity);
    }

    private void setFlywheelVelocity(double rpm) {
        // TODO: Verify the ticks per revolution for your specific flywheel motors.
        // This is 28 for a goBILDA 5202 series motor.
        double ticksPerRevolution = 28.0;
        double ticksPerSecond = rpm * (ticksPerRevolution / 60.0);
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    /** Toggles the launcher flywheels on or off. */
    public void toggleLauncher() {
        this.isLauncherOn = !this.isLauncherOn;
    }

    public boolean isLauncherOn() { return this.isLauncherOn; }
    public void increaseManualOffset() { this.manualVelocityOffset += MANUAL_VELOCITY_INCREMENT; }
    public void decreaseManualOffset() { this.manualVelocityOffset -= MANUAL_VELOCITY_INCREMENT; }
    public void resetManualOffset() { this.manualVelocityOffset = 0.0; }
    public double getCurrentTargetVelocity() { return isLauncherOn ? this.currentTargetVelocity : 0.0; }

    /** Stops the flywheels. */
    public void stop() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }
}
