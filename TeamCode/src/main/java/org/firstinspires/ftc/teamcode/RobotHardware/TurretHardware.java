package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Manages the robot's turret, which is responsible for aiming the launcher.
 * This class includes logic for automatic aiming, manual control, and calibration.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * This is a complex subsystem that requires careful calibration to work reliably.
 *
 * 1. Physical Constants: `MOTOR_TICKS_PER_REV` and `GEAR_RATIO` must match your
 *    physical robot. These are critical for converting between motor ticks and real-world
 *    degrees, which is essential for accurate aiming.
 *
 * 2. PIDF Tuning: The `TURRET_PIDF` coefficients (P, I, D, F) are the most critical
 *    tuning values. They control how the turret hones in on its target angle. These
 *    MUST be tuned using a dedicated OpMode (like TurretTuningOpMode) to find the
 *    optimal values for responsive, stable, and accurate aiming without oscillation
 *    or overshoot.
 *
 * 3. Calibration: The `calibrate()` method is designed to run the turret against a
 *    physical hard stop to find its zero position. The `CALIBRATION_POWER` and
 *    `CALIBRATION_TIME_MS` may need to be adjusted. The power should be just enough
 *    to gently push against the hard stop without stalling the motor for too long.
 *    The time should be long enough to guarantee it reaches the stop from anywhere.
 *
 * 4. Zero Point: `ZERO_POINT_DEGREES` defines what angle the turret is at after
 *    calibration. This must be measured on the physical robot. For example, if the
 *    hard stop points the turret straight back, this value would be -90 degrees.
 *
 * 5. Physical Limits: `LEFT_LIMIT_DEGREES` and `RIGHT_LIMIT_DEGREES` define the
 *    safe software limits for turret rotation. These should be set to prevent the
 *    turret from damaging itself or snagging on wires.
 * ---------------------------------------------------------------------------------
 */
public class TurretHardware {

    public enum TurretMode { AUTO, MANUAL }
    private TurretMode currentMode = TurretMode.AUTO;

    private DcMotorEx turretMotor;
    private final Follower follower;

    // --- Physical and Positional Constants ---
    // DONE: Verify these physical constants match your robot's build.
    private final double MOTOR_TICKS_PER_REV = 145.1; // For a REV HD Hex 40:1 motor
    private final double GEAR_RATIO = 82.0 / 23.0; // Example: 82-tooth turret gear, 23-tooth motor gear
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    
    // DONE: Measure and set the true angle of the turret after calibration against the hard stop.
    private final double ZERO_POINT_DEGREES = -90.0;
    
    // TODO: Set the safe software travel limits for the turret in degrees.
    private final double LEFT_LIMIT_DEGREES = 85.0;
    private final double RIGHT_LIMIT_DEGREES = -85.0;

    // --- Calculated Limits in Ticks (Do not need to be tuned directly) ---
    private final int LEFT_LIMIT_TICKS = (int) ((LEFT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int RIGHT_LIMIT_TICKS = (int) ((RIGHT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int ACCEPTABLE_TOLERANCE_TICKS = (int) (TURRET_TICKS_PER_DEGREE * 1.0); // 1.0 degree tolerance

    // --- Behavior Tuning Constants ---
    // TODO: Tune calibration power and time. Power should be low, time should be long enough to guarantee reaching the hard stop.
    private final double CALIBRATION_POWER = -0.2;
    private final int CALIBRATION_TIME_MS = 2000;
    private final double MANUAL_MOVE_SPEED = 4; // Ticks per loop for manual control increment
    
    // --- PIDF Coefficients ---
    // TODO: MUST TUNE THESE VALUES using a dedicated tuning OpMode for accurate auto-aim.
    public static final double P = 10.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final PIDFCoefficients TURRET_PIDF = new PIDFCoefficients(P, I, D, F);

    // --- State Variables ---
    private double driverNudgeDegrees = 0.0;
    public boolean isAutoAimActive = true;
    private Pose currentTargetGoal;

    public TurretHardware(Follower follower) {
        this.follower = follower;
    }

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, TURRET_PIDF);
        turretMotor.setTargetPositionTolerance(ACCEPTABLE_TOLERANCE_TICKS);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Calibrates the turret by running it against a physical hard stop to find its zero position.
     * This should be called once at the beginning of the match.
     */
    public void calibrate() {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(CALIBRATION_POWER);
        try { Thread.sleep(CALIBRATION_TIME_MS); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeToAuto();
    }

    /**
     * Main update loop for the turret. Handles automatic aiming if enabled.
     */
    public void update(GameState.Alliance alliance) {
        if (currentMode == TurretMode.AUTO && isAutoAimActive) {
            updateAutoAim(alliance);
        }
    }

    /**
     * Calculates the required turret angle to aim at the goal and commands the motor.
     */
    private void updateAutoAim(GameState.Alliance alliance) {
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;

        double absoluteAngleToGoal = calculateHeadingToGoal(alliance, robotPose);
        double relativeAngleToGoal = MathFunctions.getSmallestAngleDifference(absoluteAngleToGoal, robotPose.getHeading());
        double finalTargetAngleDegrees = Math.toDegrees(relativeAngleToGoal) + driverNudgeDegrees;
        finalTargetAngleDegrees = Math.max(RIGHT_LIMIT_DEGREES, Math.min(LEFT_LIMIT_DEGREES, finalTargetAngleDegrees));

        int targetTicks = (int) ((finalTargetAngleDegrees - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetTicks);
    }

    public Pose getTargetGoal() { return currentTargetGoal; }

    public void toggleAutoAim() {
        isAutoAimActive = !isAutoAimActive;
        if (!isAutoAimActive && currentMode == TurretMode.AUTO) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        }
    }

    /**
     * Sets the turret to manual control mode and moves it based on driver input.
     * @param input The driver's joystick input, from -1.0 to 1.0.
     */
    public void setManualControl(double input) {
        if (currentMode != TurretMode.MANUAL) {
            switchToManualMode();
        }

        int currentTarget = turretMotor.getTargetPosition();
        int newTarget = currentTarget + (int)(input * MANUAL_MOVE_SPEED);
        
        newTarget = Math.max(RIGHT_LIMIT_TICKS, Math.min(LEFT_LIMIT_TICKS, newTarget));
        turretMotor.setTargetPosition(newTarget);
    }

    /**
     * Sets the turret back to automatic (PIDF-controlled) mode.
     */
    public void setModeToAuto() {
        if (currentMode != TurretMode.AUTO) {
            currentMode = TurretMode.AUTO;
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(1.0); // Use full power for PIDF control
        }
    }

    private void switchToManualMode() {
        currentMode = TurretMode.MANUAL;
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0); // Use full power for PIDF control
    }

    public void adjustNudge(double degrees) { driverNudgeDegrees += degrees; }
    public void resetNudge() { driverNudgeDegrees = 0.0; }

    public void stop() {
        turretMotor.setPower(0);
    }

    public double getTurretCurrent() {
        return turretMotor.getCurrent(CurrentUnit.AMPS);
    }

    private double calculateHeadingToGoal(GameState.Alliance alliance, Pose robotPose) {
        currentTargetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        return Math.atan2(
                currentTargetGoal.getY() - robotPose.getY(),
                currentTargetGoal.getX() - robotPose.getX()
        );
    }
}
