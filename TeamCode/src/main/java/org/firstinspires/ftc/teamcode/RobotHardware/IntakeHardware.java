package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Manages the robot's intake mechanism.
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Motor Direction: If the intake runs backwards, change `DcMotorSimple.Direction.FORWARD`
 *    to `REVERSE`.
 * 2. Intake Power: The `INTAKE_POWER` constant can be tuned to find the optimal speed
 *    for intaking Artifacts without causing them to bounce out.
 * ---------------------------------------------------------------------------------
 */
public class IntakeHardware {

    private DcMotorEx intakeMotor;

    // TODO: Tune this power level for reliable intaking.
    private static final double INTAKE_POWER = 1.0;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop(); // Ensure motor is off at init
    }

    /** Runs the intake inwards to collect Artifacts. */
    public void run() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /** Reverses the intake to eject Artifacts. */
    public void reverse() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    /** Stops the intake motor. */
    public void stop() {
        intakeMotor.setPower(0.0);
    }

    /** Returns the current drawn by the intake motor in Amps. Useful for stall detection. */
    public double getIntakeCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
