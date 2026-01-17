package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherHardware {

    private DcMotorEx leftFlywheel = null;
    private DcMotorEx rightFlywheel = null;

    // These should be tuned for your robot
    public static final double TARGET_RPM = 2500; // The desired RPM for scoring
    public static final double RPM_TOLERANCE = 50; // Allowable error in RPM
    public static final double MAX_RPM = 6000; // Maximum RPM for the launcher

    public static final double TICKS_PER_REV = 28;

    public void init(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotor.Direction.FORWARD);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Set custom PIDF coefficients if default tuning isn't good enough
        // PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        // leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        // rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
    }

    // --- Public Methods ---

    public void setTargetRPM(double rpm) {
        // This is a placeholder as TARGET_RPM is final. We need to change the implementation.
        // The spinUp method should accept the RPM directly.
    }

// CORRECTED approach:
// In LauncherHardware.java, change the spinUp() method to take a parameter.

    /** Spins up the flywheels to the default target RPM. Used by ActionManager. */
    public void spinUp() {
        spinUp(TARGET_RPM); // Calls the other spinUp method with the default value
    }

    /** Spins up the flywheels to a specific target RPM. Used by Classic TeleOp. */
    public void spinUp(double targetRPM) {
        double targetVelocity_TPS = (targetRPM * TICKS_PER_REV) / 60.0;
        rightFlywheel.setVelocity(targetVelocity_TPS);
        leftFlywheel.setVelocity(targetVelocity_TPS);
    }

    /** Reverses the launcher to clear jams. */
    public void reverse() {
        // Use a low, constant velocity for reverse
        rightFlywheel.setVelocity(-1000);
        leftFlywheel.setVelocity(-1000);
    }

    /** Stops the flywheels. */
    public void stop() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    /** Checks if both flywheels are at the target speed within tolerance. */
    public boolean isAtTargetSpeed(double targetRPM) {
        double leftError = Math.abs(targetRPM - getLeftFlywheelRPM());
        double rightError = Math.abs(targetRPM - getRightFlywheelRPM());
        return (leftError < RPM_TOLERANCE) && (rightError < RPM_TOLERANCE);
    }

    // And the default version for the ActionManager
    public boolean isAtTargetSpeed() {
        return isAtTargetSpeed(TARGET_RPM);
    }

    public double getLeftFlywheelRPM() {
        return (leftFlywheel.getVelocity() * 60.0 / TICKS_PER_REV);
    }

    public double getRightFlywheelRPM(){
        return (rightFlywheel.getVelocity() * 60.0 / TICKS_PER_REV);
    }
}
