package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware class for the color-sorting diverter gate. This class uses an AXON servo
 * with its analog position feedback wire connected to an analog port on the Control Hub.
 * This allows us to read the servo's actual physical position to detect jams.
 */
public class DiverterHardware {

    private Servo diverterServo;
    private AnalogInput positionSensor;

    // --- These values MUST be calibrated for your specific robot! ---
    // To calibrate: 
    // 1. In your TeleOp, add telemetry to display `positionSensor.getVoltage()`.
    // 2. Manually move the servo to its full GREEN position and record the voltage.
    // 3. Manually move the servo to its full PURPLE position and record the voltage.
    // 4. The neutral voltage should be roughly halfway between the two.
    private static final double VOLTAGE_AT_GREEN_POS = 0.5;  // Placeholder
    private static final double VOLTAGE_AT_PURPLE_POS = 2.5; // Placeholder

    // The servo's commanded positions (0.0 to 1.0).
    public static final double GREEN_POSITION = 0.2;  // Placeholder
    public static final double PURPLE_POSITION = 0.8; // Placeholder
    public static final double NEUTRAL_POSITION = 0.5;

    public enum GatePosition { GREEN, PURPLE, NEUTRAL }
    private GatePosition lastCommandedPosition = GatePosition.NEUTRAL;

    public void init(HardwareMap hardwareMap) {
        diverterServo = hardwareMap.get(Servo.class, "diverterServo");
        positionSensor = hardwareMap.get(AnalogInput.class, "diverterPosition");
    }

    /**
     * Commands the diverter gate to a new position.
     */
    public void setPosition(GatePosition position) {
        lastCommandedPosition = position;
        switch (position) {
            case GREEN:
                diverterServo.setPosition(GREEN_POSITION);
                break;
            case PURPLE:
                diverterServo.setPosition(PURPLE_POSITION);
                break;
            case NEUTRAL:
                diverterServo.setPosition(NEUTRAL_POSITION);
                break;
        }
    }

    /**
     * Reads the analog sensor and converts its voltage to a 0.0-1.0 scale
     * representing the servo's approximate actual physical position.
     * @return The servo's actual position, from 0.0 (Green side) to 1.0 (Purple side).
     */
    public double getActualPosition() {
        double voltage = positionSensor.getVoltage();
        // Linearly scales the voltage reading to the 0.0-1.0 servo range.
        double scaledPosition = (voltage - VOLTAGE_AT_GREEN_POS) / (VOLTAGE_AT_PURPLE_POS - VOLTAGE_AT_GREEN_POS);
        // Clamp the result to the valid 0.0-1.0 range to handle noise or calibration errors.
        return Math.max(0.0, Math.min(1.0, scaledPosition));
    }

    /**
     * Checks if the servo is physically stuck and unable to reach its commanded position.
     * This is the key to inferring if a track is full and a pixel is blocking the gate.
     * @return True if the servo appears to be jammed.
     */
    public boolean isStuck() {
        // Define a tolerance. If the actual position is further than this from the
        // commanded position after a brief moment, we consider it "stuck". 
        // This value is a percentage of the total range and may need tuning.
        final double STUCK_TOLERANCE = 0.10; // 10% difference
        return Math.abs(getActualPosition() - getCommandedPositionAsDouble()) > STUCK_TOLERANCE;
    }

    private double getCommandedPositionAsDouble() {
        switch (lastCommandedPosition) {
            case GREEN:
                return GREEN_POSITION;
            case PURPLE:
                return PURPLE_POSITION;
            default:
                return NEUTRAL_POSITION;
        }
    }
}
