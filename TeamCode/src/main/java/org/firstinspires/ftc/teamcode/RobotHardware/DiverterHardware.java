package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware class for the color-sorting diverter gate servo.
 */
public class DiverterHardware {

    private Servo diverterServo;

    // Define the servo positions for each track. These are placeholders and must be tuned.
    public static final double PURPLE_TRACK_POSITION = 0.0;
    public static final double GREEN_TRACK_POSITION = 1.0;
    public static final double NEUTRAL_POSITION = 0.5; // Neutral position for clearing jams

    public enum GatePosition {
        PURPLE,  // Directs artifacts to the left track
        GREEN,   // Directs artifacts to the right track
        NEUTRAL
    }

    public void init(HardwareMap hardwareMap) {
        diverterServo = hardwareMap.get(Servo.class, "diverter");
    }

    /**
     * Sets the diverter gate to the specified position.
     * @param position The target position (PURPLE or GREEN).
     */
    public void setPosition(GatePosition position) {
        switch (position) {
            case PURPLE:
                diverterServo.setPosition(PURPLE_TRACK_POSITION);
                break;
            case GREEN:
                diverterServo.setPosition(GREEN_TRACK_POSITION);
                break;
            case NEUTRAL:
                diverterServo.setPosition(NEUTRAL_POSITION);
                break;
        }
    }

    /** Sets the diverter to the neutral position, typically for clearing jams. */
    public void setNeutral() {
        setPosition(GatePosition.NEUTRAL);
    }
}
