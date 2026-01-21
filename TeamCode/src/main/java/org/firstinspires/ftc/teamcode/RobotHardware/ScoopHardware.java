package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Manages the robot's scoop mechanism, which lifts a single Artifact into the launcher.
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Servo Direction: If the scoop moves down when you command it to go up, change
 *    `Servo.Direction.REVERSE` to `FORWARD`.
 * 2. Positions: The `UP_POSITION` and `DOWN_POSITION` constants must be tuned to match
 *    the physical limits of your scoop mechanism. The `UP_POSITION` should be high
 *    enough to reliably contact the flywheel, and the `DOWN_POSITION` should be low
 *    enough to be out of the way of the transfer system.
 * ---------------------------------------------------------------------------------
 */
public class ScoopHardware {

    private Servo scoopServo;

    // TODO: Tune these servo positions for your specific robot geometry.
    private static final double UP_POSITION = 0.8;
    private static final double DOWN_POSITION = 0.3;

    public void init(HardwareMap hardwareMap){
        scoopServo = hardwareMap.get(Servo.class, "scoop");
        scoopServo.setDirection(Servo.Direction.REVERSE);
        down(); // Start with the scoop in the down position.
    }

    /** Moves the scoop to the 'up' position to lift an Artifact into the flywheel. */
    public void up(){
        scoopServo.setPosition(UP_POSITION);
    }

    /** Moves the scoop to the 'down' position, its resting state. */
    public void down(){
        scoopServo.setPosition(DOWN_POSITION);
    }

    /** The stop command for a servo should be to go to a known resting state. */
    public void stop(){
        down();
    }
}
