package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Manages the robot's turret, which is responsible for aiming the launcher.
 * This class includes logic for automatic aiming, manual control, and a non-blocking
 * calibration sequence.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Physical Constants: `MOTOR_TICKS_PER_REV` and `GEAR_RATIO` must match your physical robot.
 * 2. PIDF Tuning: The `TURRET_PIDF` coefficients MUST be tuned using a dedicated OpMode
 *    (like TurretTuningOpMode) for accurate aiming.
 * 3. Calibration: The `calibrate()` method runs the turret against a hard stop. The `CALIBRATION_POWER`
 *    and `CALIBRATION_TIME_MS` may need to be adjusted.
 * 4. Zero Point & Limits: `ZERO_POINT_DEGREES` and the software limits must be measured
 *    and set based on the physical robot.
 * ---------------------------------------------------------------------------------
 */
public class TurretHardware {

    public enum TurretMode { AUTO, MANUAL }
    private TurretMode currentMode = TurretMode.AUTO;

    public enum CalibrationState { IDLE, CALIBRATING, CALIBRATED }
    private CalibrationState calibrationState = CalibrationState.IDLE;
    private final ElapsedTime calibrationTimer = new ElapsedTime();

    private DcMotorEx turretMotor;
    private final Follower follower;

    // --- Constants ---
    private final double MOTOR_TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 82.0 / 23.0;
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final double ZERO_POINT_DEGREES = -90.0;
    private final double LEFT_LIMIT_DEGREES = 85.0;
    private final double RIGHT_LIMIT_DEGREES = -85.0;
    private final double CENTER_POINT_DEGREES = 0;
    private final int LEFT_LIMIT_TICKS = (int) ((LEFT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int RIGHT_LIMIT_TICKS = (int) ((RIGHT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int ACCEPTABLE_TOLERANCE_TICKS = (int) (TURRET_TICKS_PER_DEGREE * 1.0);
    private final double CALIBRATION_POWER = -0.4;
    private final int CALIBRATION_TIME_MS = 2000;
    private final double MANUAL_MOVE_SPEED = 4;

    private double P = 75.0, I = 4.0, D = 7.5, F = 5.0;
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
        turretMotor.setVelocityPIDFCoefficients(P, I, D, F);
        turretMotor.setTargetPositionTolerance(ACCEPTABLE_TOLERANCE_TICKS);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
    }

    /**
     * Starts the non-blocking calibration sequence.
     */
    public void calibrate() {
        if (calibrationState == CalibrationState.IDLE) {
            calibrationState = CalibrationState.CALIBRATING;
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setPower(CALIBRATION_POWER);
            calibrationTimer.reset();
        }
    }

    public void update(GameState.Alliance alliance) {
        // Handle the calibration state machine
        if (calibrationState == CalibrationState.CALIBRATING) {
            if (calibrationTimer.milliseconds() >= CALIBRATION_TIME_MS) {
                turretMotor.setPower(0);
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setModeToAuto();
                calibrationState = CalibrationState.CALIBRATED;
                updateAutoAim(alliance);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return; // Don't allow other actions during calibration
        }

        // Normal update logic
        if (currentMode == TurretMode.AUTO && isAutoAimActive) {
            updateAutoAim(alliance);
        }
    }

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

    public boolean isCalibrated() {
        return calibrationState == CalibrationState.CALIBRATED;
    }

    public Pose getTargetGoal() { return currentTargetGoal; }

    public void toggleAutoAim() {
        isAutoAimActive = !isAutoAimActive;
        if (!isAutoAimActive && currentMode == TurretMode.AUTO) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        }
    }

    public void setManualControl(double input) {
        if (currentMode != TurretMode.MANUAL) {
            switchToManualMode();
        }

        int currentTarget = turretMotor.getTargetPosition();
        int newTarget = currentTarget + (int)(input * MANUAL_MOVE_SPEED);

        newTarget = Math.max(RIGHT_LIMIT_TICKS, Math.min(LEFT_LIMIT_TICKS, newTarget));
        turretMotor.setTargetPosition(newTarget);
    }

    public void setModeToAuto() {
        if (currentMode != TurretMode.AUTO) {
            currentMode = TurretMode.AUTO;
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(1.0);
        }
    }

    private void switchToManualMode() {
        currentMode = TurretMode.MANUAL;
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);
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

//
//package org.firstinspires.ftc.teamcode.RobotHardware;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.MathFunctions;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//
///**
// * Manages the robot's turret, which is responsible for aiming the launcher.
// * This class includes logic for automatic aiming, manual control, a non-blocking
// * calibration sequence, and a stall detection safety feature.
// * <p>
// * ---------------------------------------------------------------------------------
// * --- TESTING, TUNING, AND CONFIGURATION ---
// * ---------------------------------------------------------------------------------
// * 1. Physical Constants: `MOTOR_TICKS_PER_REV` and `GEAR_RATIO` must match your physical robot.
// * 2. PIDF Tuning: The `TURRET_PIDF` coefficients MUST be tuned for accurate aiming.
// * 3. Calibration: `CALIBRATION_POWER` and `CALIBRATION_TIME_MS` may need adjustment.
// * 4. Stall Detection: `STALL_CURRENT_AMPS` is a critical safety value. Set it just above
// *    the normal operating current to prevent motor burnout.
// * 5. Zero Point & Limits: `ZERO_POINT_DEGREES` and software limits must be measured.
// * ---------------------------------------------------------------------------------
// */
//public class TurretHardware {
//
//    public enum TurretMode { AUTO, MANUAL, CENTER_AND_OFF }
//    private TurretMode currentMode = TurretMode.AUTO;
//
//    public enum SystemState {
//        NEEDS_CALIBRATION,
//        CALIBRATING,
//        RUNNING,
//        STALLED, // New state for when the motor is stalled
//        CENTERING // New state for the centering sequence
//    }
//    private SystemState currentState = SystemState.NEEDS_CALIBRATION;
//    private final ElapsedTime stateTimer = new ElapsedTime();
//
//    private DcMotorEx turretMotor;
//    private final Follower follower;
//
//    // --- Constants ---
//    // Physical
//    private final double MOTOR_TICKS_PER_REV = 145.1;
//    private final double GEAR_RATIO = 82.0 / 23.0;
//    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
//    // Limits & Calibration
//    private final double ZERO_POINT_DEGREES = -90.0;
//    private final double LEFT_LIMIT_DEGREES = 85.0;
//    private final double RIGHT_LIMIT_DEGREES = -85.0;
//    private final int LEFT_LIMIT_TICKS = (int) ((LEFT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
//    private final int RIGHT_LIMIT_TICKS = (int) ((RIGHT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
//    private final double CALIBRATION_POWER = -0.3; // Increased slightly for reliability
//    private final int CALIBRATION_TIME_MS = 2000;
//    // Performance & Safety
//    private final int ACCEPTABLE_TOLERANCE_TICKS = (int) (TURRET_TICKS_PER_DEGREE * 1.0);
//    private final double MANUAL_MOVE_SPEED = 15.0; // Increased for more responsive manual control
//    private final double STALL_CURRENT_AMPS = 4.0; // TODO: Tune this critical safety value
//    private final double STALL_COOLDOWN_SECONDS = 3.0;
//
//    // PIDF
//    public static final double P = 10.0;
//    public static final double I = 0.0;
//    public static final double D = 0.0;
//    public static final double F = 0.0;
//    public static final PIDFCoefficients TURRET_PIDF = new PIDFCoefficients(P, I, D, F);
//
//    // State
//    private double driverNudgeDegrees = 0.0;
//    public boolean isAutoAimActive = true;
//    private Pose currentTargetGoal;
//
//    public TurretHardware(Follower follower) {
//        this.follower = follower;
//    }
//
//    public void init(HardwareMap hardwareMap) {
//        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, TURRET_PIDF);
//        turretMotor.setTargetPositionTolerance(ACCEPTABLE_TOLERANCE_TICKS);
//
//        // Don't reset encoder here; wait for calibration
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretMotor.setPower(0);
//    }
//
//    /**
//     * Starts the non-blocking calibration sequence. Can be called at any time.
//     */
//    public void calibrate() {
//        // Allow recalibration from any state
//        currentState = SystemState.CALIBRATING;
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretMotor.setPower(CALIBRATION_POWER);
//        stateTimer.reset();
//    }
//
//    public void update(GameState.Alliance alliance) {
//        // Main state machine for the turret's system status
//        switch (currentState) {
//            case CALIBRATING:
//                if (stateTimer.milliseconds() >= CALIBRATION_TIME_MS) {
//                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    turretMotor.setTargetPosition(0);
//                    turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    turretMotor.setPower(1.0);
//                    currentState = SystemState.RUNNING;
//                    setModeToAuto(); // Default to auto-aim after calibrating
//                }
//                return; // Block other actions during calibration
//
//            case STALLED:
//                // Motor is in a cooldown period.
//                if (stateTimer.seconds() > STALL_COOLDOWN_SECONDS) {
//                    currentState = SystemState.RUNNING; // Attempt to recover
//                    turretMotor.setPower(1.0); // Re-enable RUN_TO_POSITION power
//                }
//                return; // Block other actions during cooldown
//
//            case NEEDS_CALIBRATION:
//                // Turret has not been calibrated. Only manual control is allowed.
//                // The handleManualControl method will handle this.
//                break;
//
//            case CENTERING:
//                if (!turretMotor.isBusy()) {
//                    // We have arrived at the center position. Now power off.
//                    turretMotor.setPower(0);
//                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    currentState = SystemState.RUNNING; // Return to normal state, but powered off
//                }
//                break;
//
//            case RUNNING:
//                // --- Stall Detection ---
//                if (turretMotor.getCurrent(CurrentUnit.AMPS) > STALL_CURRENT_AMPS && !turretMotor.isBusy()) {
//                    turretMotor.setPower(0); // Immediately cut power
//                    currentState = SystemState.STALLED;
//                    stateTimer.reset();
//                    return; // Exit to begin cooldown
//                }
//
//                // --- Normal Update Logic ---
//                if (currentMode == TurretMode.AUTO && isAutoAimActive) {
//                    updateAutoAim(alliance);
//                }
//                break;
//        }
//    }
//
//    private void updateAutoAim(GameState.Alliance alliance) {
//        if (currentState != SystemState.RUNNING) return; // Can't auto-aim if not calibrated/running
//        Pose robotPose = follower.getPose();
//        if (robotPose == null) return;
//
//        double absoluteAngleToGoal = calculateHeadingToGoal(alliance, robotPose);
//        double relativeAngleToGoal = MathFunctions.getSmallestAngleDifference(absoluteAngleToGoal, robotPose.getHeading());
//        double finalTargetAngleDegrees = Math.toDegrees(relativeAngleToGoal) + driverNudgeDegrees;
//
//        setTargetAngle(finalTargetAngleDegrees);
//    }
//
//    private void setTargetAngle(double degrees) {
//        // Clamp the angle to the physical limits of the turret
//        degrees = Math.max(RIGHT_LIMIT_DEGREES, Math.min(LEFT_LIMIT_DEGREES, degrees));
//        int targetTicks = (int) ((degrees - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
//        turretMotor.setTargetPosition(targetTicks);
//    }
//
//    public boolean isCalibrated() {
//        return currentState == SystemState.RUNNING || currentState == SystemState.STALLED;
//    }
//
//    public void setManualControl(double input) {
//        // Allow manual control even if not calibrated
//        if (currentMode != TurretMode.MANUAL) {
//            setModeToManual();
//        }
//
//        // If not calibrated, we can't use RUN_TO_POSITION. Use direct power.
//        if (currentState == SystemState.NEEDS_CALIBRATION) {
//            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            turretMotor.setPower(input * 0.5); // Use a scaled power for manual control
//            return;
//        }
//
//        // If calibrated, use smooth position-based manual control
//        int currentTarget = turretMotor.getTargetPosition();
//        int newTarget = currentTarget + (int)(input * MANUAL_MOVE_SPEED);
//
//        newTarget = Math.max(RIGHT_LIMIT_TICKS, Math.min(LEFT_LIMIT_TICKS, newTarget));
//        turretMotor.setTargetPosition(newTarget);
//    }
//
//    public void setModeToAuto() {
//        currentMode = TurretMode.AUTO;
//        if (isCalibrated()) {
//            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turretMotor.setPower(1.0);
//        }
//    }
//
//    public void setModeToManual() {
//        currentMode = TurretMode.MANUAL;
//        if (isCalibrated()) {
//            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
//            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turretMotor.setPower(1.0);
//        }
//    }
//
//    public void centerAndPowerOff() {
//        if (!isCalibrated()) return; // Can't do this if we don't know where center is.
//
//        currentMode = TurretMode.CENTER_AND_OFF;
//        currentState = SystemState.CENTERING;
//
//        setTargetAngle(0); // Target the center
//        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turretMotor.setPower(0.8); // Use slightly less power for centering
//    }
//
//    public void toggleAutoAim() { /* ... (no changes needed) ... */ }
//    public void adjustNudge(double degrees) { /* ... (no changes needed) ... */ }
//    public void resetNudge() { /* ... (no changes needed) ... */ }
//    public void stop() { /* ... (no changes needed) ... */ }
//    public double getTurretCurrent() { /* ... (no changes needed) ... */ }
//    private double calculateHeadingToGoal(GameState.Alliance alliance, Pose robotPose) { /* ... (no changes needed) ... */ }
//    public boolean isAutoAimActive() { return isAutoAimActive; }
//    public String getSystemState() { return currentState.toString(); }
//}