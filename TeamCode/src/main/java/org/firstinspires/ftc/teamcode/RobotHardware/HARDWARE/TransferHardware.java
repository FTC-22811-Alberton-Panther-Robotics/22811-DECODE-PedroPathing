package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferHardware {
    public CRServo transferServo;
    private static final double TRANSFER_POWER = 1.0;

    public void init(HardwareMap hardwareMap){
        transferServo = hardwareMap.get(CRServo.class, "transfer");
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);
        stop(); // Ensure it's off at init
    }

    // --- Public Methods ---

    /** Runs the transfer to move pixels towards the launcher. */
    public void run() {
        transferServo.setPower(TRANSFER_POWER);
    }

    /** Reverses the transfer. */
    public void reverse() {
        transferServo.setPower(-TRANSFER_POWER);
    }

    /** Stops the transfer. */
    public void stop() {
        transferServo.setPower(0.0);
    }
}
