package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Manages the robot's Artifact diverter mechanism. This class is a self-contained module
 * responsible for both manual and automatic control of the diverter gate.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * This is a complex subsystem that requires careful calibration to work reliably.
 *
 * 1. Servo Positions: If using a non-continuous servo, the `GREEN_POSITION`,
 *    `PURPLE_POSITION`, and `NEUTRAL_POSITION` constants must be tuned to match the
 *    physical positions you want the servo to move to.
 *
 * 2. Analog Voltage Feedback: This allows the robot to know the servo's *actual*
 *    position. To calibrate `VOLTAGE_AT_GREEN_POS` and `VOLTAGE_AT_PURPLE_POS`:
 *      a. Add `telemetry.addData("Diverter Voltage", robot.diverter.positionSensor.getVoltage());`
 *         to your TeleOp's `loop()` method.
 *      b. Manually push the diverter gate all the way to the green-sorting side and
 *         record the voltage value from telemetry. This is your `VOLTAGE_AT_GREEN_POS`.
 *      c. Repeat for the purple side to find `VOLTAGE_AT_PURPLE_POS`.
 *
 * 3. Stuck Tolerance: The `STUCK_TOLERANCE` value in `isStuck()` determines how far
 *    the servo's actual position can be from its commanded position before we consider
 *    it jammed. Tune this value to be sensitive enough to detect jams without giving
 *    false positives during normal movement.
 *
 * 4. Counting Logic: The logic that infers which Artifact was sorted when a jam is
 *    detected is based on a physical assumption. This needs to be tested with the real
 *    robot to confirm it behaves as expected.
 * ---------------------------------------------------------------------------------
 */
public class DiverterHardware {

    // --- Hardware Devices ---
    private Servo diverter;
    private AnalogInput positionSensor;
    private LimelightBallDetector limelight;

    // --- State Variables ---
    private int greenArtifactCount = 0;
    private int purpleArtifactCount = 0;

    // --- Calibration Constants ---
    private static final double VOLTAGE_AT_GREEN_POS = 0.5;
    private static final double VOLTAGE_AT_PURPLE_POS = 2.5;

    // Allowable voltage difference between commanded and actual position before the servo is
    // considered stuck
    final double STUCK_TOLERANCE = 0.20;

    // --- Servo Positions ---

    public static final double GREEN_POSITION = 0.0;
    public static final double PURPLE_POSITION = 1.0;
    public static final double NEUTRAL_POSITION = 0.5;

    public enum GatePosition { GREEN, PURPLE, NEUTRAL }
    private GatePosition lastCommandedPosition = GatePosition.NEUTRAL;

    public void init(HardwareMap hardwareMap, LimelightBallDetector limelight) {
        this.limelight = limelight;
        diverter = hardwareMap.get(Servo.class, "diverter");
        positionSensor = hardwareMap.get(AnalogInput.class, "diverterPosition");
    }

    public void update() {
        GatePosition targetPosition;
        LimelightBallDetector.DetectedBall detected = limelight.getDetectedColor();

        switch (detected) {
            case GREEN:
                targetPosition = GatePosition.GREEN;
                break;
            case PURPLE:
                targetPosition = GatePosition.PURPLE;
                break;
            case NONE:
            default:
                targetPosition = GatePosition.NEUTRAL;
                break;
        }

        if ((targetPosition == GatePosition.GREEN && greenArtifactCount >= 2) || 
            (targetPosition == GatePosition.PURPLE && purpleArtifactCount >= 2)) {
            targetPosition = GatePosition.NEUTRAL;
        }

        setPosition(targetPosition);

        if (isStuck()) {
            if (targetPosition == GatePosition.GREEN) purpleArtifactCount++;
            else if (targetPosition == GatePosition.PURPLE) greenArtifactCount++;
            setPosition(GatePosition.NEUTRAL);
        } else {
            if (targetPosition == GatePosition.GREEN) greenArtifactCount++;
            else if (targetPosition == GatePosition.PURPLE) purpleArtifactCount++;
        }
    }

    public void setPosition(GatePosition position) {
        lastCommandedPosition = position;
        switch (position) {
            case GREEN: diverter.setPosition(GREEN_POSITION); break;
            case PURPLE: diverter.setPosition(PURPLE_POSITION); break;
            case NEUTRAL: diverter.setPosition(NEUTRAL_POSITION); break;
        }
    }

    public double getActualPosition() {
        double voltage = positionSensor.getVoltage();
        double scaledPosition = (voltage - VOLTAGE_AT_GREEN_POS) / (VOLTAGE_AT_PURPLE_POS - VOLTAGE_AT_GREEN_POS);
        return Math.max(0.0, Math.min(1.0, scaledPosition));
    }

    public boolean isStuck() {
        return Math.abs(getActualPosition() - getCommandedPositionAsDouble()) > STUCK_TOLERANCE;
    }

    // --- Artifact Count Management ---
    public void decrementGreenArtifactCount() { greenArtifactCount = Math.max(0, greenArtifactCount - 1); }
    public void decrementPurpleArtifactCount() { purpleArtifactCount = Math.max(0, purpleArtifactCount - 1); }
    public int getGreenArtifactCount() { return greenArtifactCount; }
    public int getPurpleArtifactCount() { return purpleArtifactCount; }

    private double getCommandedPositionAsDouble() {
        switch (lastCommandedPosition) {
            case GREEN: return GREEN_POSITION;
            case PURPLE: return PURPLE_POSITION;
            default: return NEUTRAL_POSITION;
        }
    }
}
