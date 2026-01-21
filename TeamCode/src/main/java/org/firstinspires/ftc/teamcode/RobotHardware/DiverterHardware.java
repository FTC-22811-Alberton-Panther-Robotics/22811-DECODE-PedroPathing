package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Optional;

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
 * 4. Region of Interest (ROI): To prevent the Limelight from seeing Artifacts already
 *    inside the robot, we only look for new Artifacts in a specific vertical slice
 *    of the camera view. To tune `ROI_MIN_Y` and `ROI_MAX_Y`:
 *      a. In the Limelight web interface, look at the camera stream.
 *      b. The Y-coordinates range from -240 (top edge) to 240 (bottom edge).
 *      c. Find the Y-pixel values that define a band that is *only* outside your robot.
 *         `ROI_MIN_Y` should be the top of this band, and `ROI_MAX_Y` the bottom.
 *
 * 5. Counting Logic: The logic that infers which Artifact was sorted when a jam is
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
    // TODO: Calibrate the voltage value for the green position.
    private static final double VOLTAGE_AT_GREEN_POS = 0.5;
    // TODO: Calibrate the voltage value for the purple position.
    private static final double VOLTAGE_AT_PURPLE_POS = 2.5;

    public static final double GREEN_POSITION = 0.0;
    public static final double PURPLE_POSITION = 1.0;
    public static final double NEUTRAL_POSITION = 0.5;

    // TODO: Tune the Region of Interest (ROI) to only see Artifacts outside the robot.
    // Y-pixel coordinates for the detection ROI. Range is from -240.0 (top) to 240.0 (bottom).
    private static final double ROI_MIN_Y = -100.0;
    private static final double ROI_MAX_Y = 100.0;

    public enum GatePosition { GREEN, PURPLE, NEUTRAL }
    private GatePosition lastCommandedPosition = GatePosition.NEUTRAL;

    public void init(HardwareMap hardwareMap, LimelightBallDetector limelight) {
        this.limelight = limelight;
        diverter = hardwareMap.get(Servo.class, "diverter");
        positionSensor = hardwareMap.get(AnalogInput.class, "diverterPosition");
    }

    public void update() {
        limelight.setPipeline(1); // Green Artifact pipeline
        Optional<LLResultTypes.DetectorResult> greenArtifact = limelight.getLargestArtifactInROI(ROI_MIN_Y, ROI_MAX_Y);

        limelight.setPipeline(2); // Purple Artifact pipeline
        Optional<LLResultTypes.DetectorResult> purpleArtifact = limelight.getLargestArtifactInROI(ROI_MIN_Y, ROI_MAX_Y);

        GatePosition targetPosition = GatePosition.NEUTRAL;

        if (greenArtifact.isPresent() && purpleArtifact.isPresent()) {
            if (greenArtifact.get().getTargetArea() > purpleArtifact.get().getTargetArea()) {
                targetPosition = GatePosition.GREEN;
            } else {
                targetPosition = GatePosition.PURPLE;
            }
        } else if (greenArtifact.isPresent()) {
            targetPosition = GatePosition.GREEN;
        } else if (purpleArtifact.isPresent()) {
            targetPosition = GatePosition.PURPLE;
        }

        if (targetPosition == GatePosition.GREEN && greenArtifactCount >= 2) {
            targetPosition = GatePosition.NEUTRAL;
        } else if (targetPosition == GatePosition.PURPLE && purpleArtifactCount >= 2) {
            targetPosition = GatePosition.NEUTRAL;
        }

        setPosition(targetPosition);

        // TODO: Test and verify this Artifact counting logic. The inference made when the gate
        // is stuck might need to be adjusted based on physical robot behavior.
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
        // TODO: Tune this tolerance. A value too high will not detect jams; too low will give false positives.
        final double STUCK_TOLERANCE = 0.10;
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
