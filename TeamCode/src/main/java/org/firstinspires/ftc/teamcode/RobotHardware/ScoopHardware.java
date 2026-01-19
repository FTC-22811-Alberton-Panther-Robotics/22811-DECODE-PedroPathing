package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoopHardware {

    public Servo ballScoop;

    // Define constants for the servo positions for clarity and easy tuning.
    private static final double UP_POSITION = 0.8;
    private static final double DOWN_POSITION = 0.3;

    public void init(HardwareMap hardwaremap){
        ballScoop = hardwaremap.get(Servo.class, "scoop");

        // CORRECTED: Reverse the servo direction so that 1.0 is UP and 0.0 is DOWN.
        ballScoop.setDirection(Servo.Direction.REVERSE);

        // REMOVED: Do not command servo movement during initialization.
    }

    /** Moves the scoop to the 'up' position to interact with a pixel. */
    public void ballUp(){
        ballScoop.setPosition(UP_POSITION);
    }

    /** Moves the scoop to the 'down' position, its resting state. */
    public void ballDown(){
        ballScoop.setPosition(DOWN_POSITION);
    }

    /** The stop command for a servo should be to go to a known resting state. */
    public void stop(){
        ballDown();
    }
}
