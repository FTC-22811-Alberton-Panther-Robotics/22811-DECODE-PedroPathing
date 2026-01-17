package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherHardware {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private final Follower follower;
    private final TurretHardware turret;

    // --- Tunable Constants for Automatic Speed Control ---
    public static double MIN_SHOT_DISTANCE = 36; // Inches
    public static double MAX_SHOT_DISTANCE = 120; // Inches
    public static double MIN_SHOT_VELOCITY = 2000; // RPM
    public static double MAX_SHOT_VELOCITY = 3500; // RPM
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

        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotor.Direction.FORWARD);

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

        double slope = (MAX_SHOT_VELOCITY - MIN_SHOT_VELOCITY) / (MAX_SHOT_DISTANCE - MIN_SHOT_DISTANCE);
        double calculatedVelocity = MIN_SHOT_VELOCITY + slope * (distanceToGoal - MIN_SHOT_DISTANCE);
        calculatedVelocity = Math.max(MIN_SHOT_VELOCITY, Math.min(MAX_SHOT_VELOCITY, calculatedVelocity));

        this.currentTargetVelocity = calculatedVelocity + manualVelocityOffset;
        setFlywheelVelocity(this.currentTargetVelocity);
    }

    private void setFlywheelVelocity(double rpm) {
        double ticksPerSecond = rpm * (28.0 / 60.0); // 28 ticks per rev for goBILDA motor
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    /** Toggles the launcher flywheels on or off. */
    public void toggleLauncher() {
        this.isLauncherOn = !this.isLauncherOn;
    }

    /** Returns whether the launcher is currently on. */
    public boolean isLauncherOn() {
        return this.isLauncherOn;
    }

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
