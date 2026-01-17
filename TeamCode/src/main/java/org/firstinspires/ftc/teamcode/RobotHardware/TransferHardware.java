package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferHardware {
    public Servo rightTransfer;
    public Servo leftTransfer;

    // CORRECTED: Renamed constants for clarity.
    private static final double RUN_POSITION = 1.0;
    private static final double STOP_POSITION = 0.0;

    public void init(HardwareMap hardwareMap){
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftTransfer = hardwareMap.get(Servo.class,"leftTransfer");

        // CORRECTED: Reverse the right servo so both servos run in the same direction with the same position values.
        rightTransfer.setDirection(Servo.Direction.REVERSE);

        stop(); // Ensure it's off at init
    }

    // --- Public Methods ---

    /** Runs the transfer to move pixels towards the launcher. */
    public void runRight() {
        rightTransfer.setPosition(RUN_POSITION);
    }
    public void runLeft() {
        leftTransfer.setPosition(RUN_POSITION);
    }
    public void LeftTransferReturn(){
        leftTransfer.setPosition(STOP_POSITION);
    }
    public void RightTransferReturn(){
        rightTransfer.setPosition(STOP_POSITION);
    }

    /** Stops both transfer servos. */
    public void stop() {
        // CORRECTED: This method now correctly stops both servos.
        leftTransfer.setPosition(STOP_POSITION);
        rightTransfer.setPosition(STOP_POSITION);
    }
}
