package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TurretHardware {

    public enum TurretMode { AUTO, MANUAL }
    private TurretMode currentMode = TurretMode.AUTO;

    private DcMotorEx turretMotor;
    private final Follower follower;

    // --- Constants ---
    // Physical and Positional Constants
    private final double MOTOR_TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 82.0 / 23.0;
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final double ZERO_POINT_DEGREES = -90.0;
    private final double LEFT_LIMIT_DEGREES = 85.0;
    private final double RIGHT_LIMIT_DEGREES = -85.0;
    private final int LEFT_LIMIT_TICKS = (int) ((LEFT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int RIGHT_LIMIT_TICKS = (int) ((RIGHT_LIMIT_DEGREES - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    private final int ACCEPTABLE_TOLERANCE_TICKS = (int) (TURRET_TICKS_PER_DEGREE * 1.0); // 1.0 degree tolerance

    // Tuning and Behavior Constants
    private final double CALIBRATION_POWER = -0.2;
    private final int CALIBRATION_TIME_MS = 2000;
    private final double MANUAL_MOVE_SPEED = 4; // Ticks per loop for manual control increment
    
    // --- PIDF Coefficients ---
    // NOTE: These are starting values and MUST be tuned for your specific robot!
    public static final double P = 10.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0; // F is often not needed for position control
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
        
        // Set the PIDF coefficients and target position tolerance.
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, TURRET_PIDF);
        turretMotor.setTargetPositionTolerance(ACCEPTABLE_TOLERANCE_TICKS);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void calibrate() {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(CALIBRATION_POWER);
        try { Thread.sleep(CALIBRATION_TIME_MS); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeToAuto();
    }

    public void update(GameState.Alliance alliance) {
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
            turretMotor.setPower(1.0); // Use full power for PIDF
        }
    }

    private void switchToManualMode() {
        currentMode = TurretMode.MANUAL;
        turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0); // Use full power for PIDF
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
