package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Manages the robot's transfer mechanism, which uses two standard servos
 * to move Artifacts from the diverter into the scoop for launching.
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Servo Directions: The two transfer servos are mounted as mirror images. One must
 *    be reversed so that giving them both the same position value makes them spin in the
 *    correct, inward direction. If they spin opposite ways, reverse one of them.
 * 2. Servo Positions: The `RUN_POSITION` and `STOP_POSITION` constants must be tuned to
 *    the needs of your physical mechanism. These represent the start and end positions
 *    for the standard servos.
 * ---------------------------------------------------------------------------------
 */
public class TransferHardware {
    public Servo rightTransfer;
    public Servo leftTransfer;

    // TODO: Tune these servo positions for your specific setup.
    private static final double RUN_POSITION = 1.0;
    private static final double STOP_POSITION = 0.0;

    public void init(HardwareMap hardwareMap){
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftTransfer = hardwareMap.get(Servo.class,"leftTransfer");

        // One servo must be reversed because they are mounted opposite to each other.
        rightTransfer.setDirection(Servo.Direction.REVERSE);

        stop(); // Ensure it's off at init
    }

    // --- Public Methods ---

    /** Runs the right transfer belt to move an Artifact into the scoop. */
    public void runRight() {
        rightTransfer.setPosition(RUN_POSITION);
    }

    /** Runs the left transfer belt to move an Artifact into the scoop. */
    public void runLeft() {
        leftTransfer.setPosition(RUN_POSITION);
    }

    /** Stops the left transfer belt. */
    public void LeftTransferReturn(){
        leftTransfer.setPosition(STOP_POSITION);
    }

    /** Stops the right transfer belt. */
    public void RightTransferReturn(){
        rightTransfer.setPosition(STOP_POSITION);
    }

    /** Stops both transfer servos. */
    public void stop() {
        leftTransfer.setPosition(STOP_POSITION);
        rightTransfer.setPosition(STOP_POSITION);
    }
}
