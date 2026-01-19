package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeHardware {

    private DcMotorEx intakeMotor = null;
    private static final double INTAKE_POWER = 1.0;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop(); // Ensure motor is off at init
    }

    // --- Public Methods ---

    /** Runs the intake inwards to collect pixels. */
    public void run() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    /** Reverses the intake to eject pixels. */
    public void reverse() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    /** Stops the intake motor. */
    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public double getIntakeCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
