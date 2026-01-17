package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretHardware {

    public enum TurretMode { AUTO, MANUAL }
    private TurretMode currentMode = TurretMode.AUTO;

    private DcMotorEx turretMotor;
    private final Follower follower;

    // --- Constants ---
    private final double MOTOR_TICKS_PER_REV = 145.1; // From goBILDA Yellow Jacket 1150 RPM motor page
    private final double GEAR_RATIO = 82.0 / 23.0;
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    
    private final double LEFT_LIMIT_DEGREES = 90.0;
    private final double RIGHT_LIMIT_DEGREES = -90.0;
    private final int CALIBRATION_TIME_MS = 500;
    private final double MANUAL_MOVE_POWER = 0.5;
    private final double AUTO_MOVE_POWER = 1.0;

    // --- State Variables ---
    private double driverNudgeDegrees = 0.0;
    public boolean isAutoAimActive = true;
    private Pose currentTargetGoal; // To share with other systems like the launcher

    public TurretHardware(Follower follower) {
        this.follower = follower;
    }

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void calibrate() {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.3);
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
        double relativeAngleDegrees = Math.toDegrees(relativeAngleToGoal);
        double finalTargetAngleDegrees = relativeAngleDegrees + driverNudgeDegrees;
        finalTargetAngleDegrees = Math.max(RIGHT_LIMIT_DEGREES, Math.min(LEFT_LIMIT_DEGREES, finalTargetAngleDegrees));
        
        // CORRECTED: Invert the tick calculation to account for the reversed motor direction.
        int targetTicks = (int) ((LEFT_LIMIT_DEGREES - finalTargetAngleDegrees) * TURRET_TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetTicks);
    }

    // --- Public Methods for OpMode Control ---

    /** Returns the pose of the goal the turret is currently aimed at. */
    public Pose getTargetGoal() { return currentTargetGoal; }

    public void toggleAutoAim() {
        this.isAutoAimActive = !this.isAutoAimActive;
        if (!isAutoAimActive && currentMode == TurretMode.AUTO) {
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
        }
    }

    public void setManualPower(double power) {
        if (currentMode != TurretMode.MANUAL) {
            currentMode = TurretMode.MANUAL;
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretMotor.setPower(power * MANUAL_MOVE_POWER);
    }

    public void setModeToAuto() {
        if (currentMode != TurretMode.AUTO) {
            currentMode = TurretMode.AUTO;
            turretMotor.setTargetPosition(turretMotor.getCurrentPosition()); 
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(AUTO_MOVE_POWER);
        }
    }

    public void adjustNudge(double degrees) { this.driverNudgeDegrees += degrees; }
    public void resetNudge() { this.driverNudgeDegrees = 0.0; }

    public void stop() {
        turretMotor.setPower(0);
    }
    
    private double calculateHeadingToGoal(GameState.Alliance alliance, Pose robotPose) {
        // This now sets the current target goal for other systems to use.
        this.currentTargetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        return Math.atan2(
                currentTargetGoal.getY() - robotPose.getY(),
                currentTargetGoal.getX() - robotPose.getX()
        );
    }
}
