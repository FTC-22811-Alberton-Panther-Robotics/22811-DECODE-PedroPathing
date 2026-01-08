package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware class for the color-sorting diverter gate servo.
 */
public class ColorDiverterHardware {

    private Servo diverterServo;

    // Define the servo positions for each track. These are placeholders and must be tuned.
    public static final double PURPLE_TRACK_POSITION = 0.0;
    public static final double GREEN_TRACK_POSITION = 1.0;

    public enum GatePosition {
        PURPLE,  // Directs artifacts to the left track
        GREEN  // Directs artifacts to the right track
    }

    public void init(HardwareMap hardwareMap) {
        diverterServo = hardwareMap.get(Servo.class, "colorDiverter");
    }

    /**
     * Sets the diverter gate to the specified position.
     * @param position The target position (PURPLE or GREEN).
     */
    public void setPosition(GatePosition position) {
        if (position == GatePosition.PURPLE) {
            diverterServo.setPosition(PURPLE_TRACK_POSITION);
        } else {
            diverterServo.setPosition(GREEN_TRACK_POSITION);
        }
    }
}
