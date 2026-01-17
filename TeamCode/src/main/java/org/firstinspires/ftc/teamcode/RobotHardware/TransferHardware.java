package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferHardware {
    public Servo rightTransfer;
    public Servo leftTransfer;

    private static final double TRANSFER_POWER = 1.0;
    private static final double RETURN = 0;

    public void init(HardwareMap hardwareMap){
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftTransfer = hardwareMap.get(Servo.class,"leftTransfer");
        stop(); // Ensure it's off at init
    }

    // --- Public Methods ---

    /** Runs the transfer to move pixels towards the launcher. */
    public void runRight() {
        rightTransfer.setPosition(TRANSFER_POWER);
    }
    public void runLeft() {
        leftTransfer.setPosition(TRANSFER_POWER);
    }
    public void LeftTransferReturn(){leftTransfer.setPosition(RETURN);}
    public void RightTransferReturn(){rightTransfer.setPosition(RETURN);}
    /** Stops the transfer. */
    public void stop() {
        rightTransfer.setPosition(0.0);
    }
}
