package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Turret PIDF Tuning", group = "00 Tuning")
public class TurretTuningOpMode extends OpMode {

    private DcMotorEx turretMotor;

    // --- Constants ---
    private final double MOTOR_TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 82.0 / 23.0;
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final double ZERO_POINT_DEGREES = -90.0;

    // --- Tuning Variables ---
    private double p = 10.0, i = 0.0, d = 0.0, f = 0.0;
    private int targetPositionTicks = 0;
    private double increment = 0.5;

    // --- Button Press Trackers ---
    private boolean dpad_up_pressed, dpad_down_pressed, dpad_left_pressed, dpad_right_pressed;
    private boolean x_pressed, y_pressed, a_pressed, b_pressed, rb_pressed, lb_pressed;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set a target position before switching to RUN_TO_POSITION
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1.0);

        telemetry.addLine("Turret PIDF Tuner Initialized.");
        telemetry.addLine("SAFETY FIRST: Ensure robot is propped up!");
        updateHelpTelemetry();
    }

    @Override
    public void loop() {
        handleInput();

        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);

        turretMotor.setTargetPosition(targetPositionTicks);

        updateTuningTelemetry();
    }

    private void handleInput() {
        // --- Target Position Control ---
        if (gamepad1.a && !a_pressed) targetPositionTicks = convertDegreesToTicks(0);   // Center
        if (gamepad1.y && !y_pressed) targetPositionTicks = convertDegreesToTicks(45);  // Left
        if (gamepad1.x && !x_pressed) targetPositionTicks = convertDegreesToTicks(-45); // Right
        if (gamepad1.b && !b_pressed) targetPositionTicks = 0; // Far Right (Calibration Point)

        // --- PIDF Value Adjustment ---
        if (gamepad1.dpad_up && !dpad_up_pressed) p += increment;
        if (gamepad1.dpad_down && !dpad_down_pressed) p -= increment;

        if (gamepad1.dpad_right && !dpad_right_pressed) d += (increment / 10); // D is usually smaller
        if (gamepad1.dpad_left && !dpad_left_pressed) d -= (increment / 10);
        
        if (gamepad1.right_bumper && !rb_pressed) i += (increment / 20); // I is usually very small
        if (gamepad1.left_bumper && !lb_pressed) i -= (increment / 20);

        updateButtonStates();
    }

    private int convertDegreesToTicks(double degrees) {
        return (int) ((degrees - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    }

    private void updateButtonStates() {
        dpad_up_pressed = gamepad1.dpad_up;
        dpad_down_pressed = gamepad1.dpad_down;
        dpad_left_pressed = gamepad1.dpad_left;
        dpad_right_pressed = gamepad1.dpad_right;
        x_pressed = gamepad1.x;
        y_pressed = gamepad1.y;
        a_pressed = gamepad1.a;
        b_pressed = gamepad1.b;
        rb_pressed = gamepad1.right_bumper;
        lb_pressed = gamepad1.left_bumper;
    }

    private void updateHelpTelemetry() {
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("A: 0 deg | Y: 45 deg | X: -45 deg");
        telemetry.addLine("D-Pad U/D: Adjust P");
        telemetry.addLine("D-Pad L/R: Adjust D");
        telemetry.addLine("Bumpers L/R: Adjust I");
        telemetry.update();
    }

    private void updateTuningTelemetry() {
        telemetry.addData("P", "%.2f", p);
        telemetry.addData("I", "%.4f", i);
        telemetry.addData("D", "%.4f", d);
        telemetry.addLine("----------------");
        telemetry.addData("Target Ticks", targetPositionTicks);
        telemetry.addData("Actual Ticks", turretMotor.getCurrentPosition());
        telemetry.addData("Error", targetPositionTicks - turretMotor.getCurrentPosition());
        telemetry.update();
    }
}
