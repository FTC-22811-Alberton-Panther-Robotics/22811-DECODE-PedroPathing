package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Turret PIDF Tuning", group = "02 Tuning")
public class TurretTuningOpMode extends OpMode {

    private DcMotorEx turretMotor;

    // --- Constants ---
    private final double MOTOR_TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 82.0 / 23.0;
    private final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final double ZERO_POINT_DEGREES = -90.0;

    // --- Tuning Variables ---
    private double p = 75.0, i = 4.0, d = 7.5, f = 5.0; // For RUN_USING_ENCODER
    private double position_p = 5.0; // For RUN_TO_POSITION
    private int targetPositionTicks = 0;
    private double p_increment = 0.5;
    private double v_increment = 5.0;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // Correctly set both sets of PIDF coefficients on every loop
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(p, i, d, f);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        turretMotor.setPositionPIDFCoefficients(position_p);

        turretMotor.setTargetPosition(targetPositionTicks);

        updateTuningTelemetry();
    }

    private void handleInput() {
        // --- Target Position Control (Gamepad 1) ---
        if (gamepad1.y) targetPositionTicks = convertDegreesToTicks(0);   // Center
        if (gamepad1.b) targetPositionTicks = convertDegreesToTicks(-45); // Left
        if (gamepad1.x) targetPositionTicks = convertDegreesToTicks(45);  // Right
        if (gamepad1.a) targetPositionTicks = convertDegreesToTicks(-90); // Far Right (Calibration Point)

        // --- Velocity PIDF Value Adjustment (Gamepad 1) ---
        if (gamepad1.dpad_up) p += v_increment;
        if (gamepad1.dpad_down) p -= v_increment;

        if (gamepad1.dpad_right) d += (v_increment / 10);
        if (gamepad1.dpad_left) d -= (v_increment / 10);
        
        if (gamepad1.right_bumper) i += (v_increment / 20);
        if (gamepad1.left_bumper) i -= (v_increment / 20);

        if (gamepad1.start) f += (v_increment/10);
        if (gamepad1.back) f -= (v_increment/10);

        // --- Position P Value Adjustment (Gamepad 2) ---
        if (gamepad2.dpad_up) position_p += p_increment;
        if (gamepad2.dpad_down) position_p -= p_increment;
    }

    private int convertDegreesToTicks(double degrees) {
        return (int) ((degrees - ZERO_POINT_DEGREES) * TURRET_TICKS_PER_DEGREE);
    }

    private void updateHelpTelemetry() {
        telemetry.addLine("--- Controls (GP1) ---");
        telemetry.addLine("Y/B/X/A: Set Target Position");
        telemetry.addLine("D-Pad U/D: Adjust Velocity P");
        telemetry.addLine("D-Pad L/R: Adjust Velocity D");
        telemetry.addLine("Bumpers: Adjust Velocity I");
        telemetry.addLine("Start/Back: Adjust Velocity F");
        telemetry.addLine("--- Controls (GP2) ---");
        telemetry.addLine("D-Pad U/D: Adjust Position P");
        telemetry.update();
    }

    private void updateTuningTelemetry() {
        telemetry.addData("Position P", "%.2f", position_p);
        telemetry.addLine("------");
        telemetry.addData("Velocity P", "%.2f", p);
        telemetry.addData("Velocity I", "%.4f", i);
        telemetry.addData("Velocity D", "%.4f", d);
        telemetry.addData("Velocity F", "%.2f", f);
        telemetry.addLine("----------------");
        telemetry.addData("Target Ticks", targetPositionTicks);
        telemetry.addData("Actual Ticks", turretMotor.getCurrentPosition());
        telemetry.addData("Error", targetPositionTicks - turretMotor.getCurrentPosition());
        telemetry.update();
    }
}
