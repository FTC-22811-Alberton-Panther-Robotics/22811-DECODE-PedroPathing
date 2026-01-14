package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class BallTransferHardware {
    public Servo leftBallPusher;
    public Servo rightBallPusher;
    private double leftForward = 1.0; //TODO: set correctly
    private double leftBack = 0.0; //TODO: set correctl
    private double rightForward = 1.0; //TODO: set correctly
    private double rightBack = 0.0; //TODO: set correctly

    public void init(HardwareMap hardwaremap){
        rightBallPusher = hardwaremap.get(Servo.class, "rightPusher");
        leftBallPusher = hardwaremap.get(Servo.class, "leftPusher");
        stop();
    }
    public void leftTransfer(){
        leftBallPusher.setPosition(leftForward);
    }
    public void leftReturn(){
        leftBallPusher.setPosition(leftBack);
    }
    public void rightTransfer() {
        rightBallPusher.setPosition(rightForward);
    }
    public void rightReturn(){
        rightBallPusher.setPosition(rightBack);
    }
    public void stop(){
        leftBallPusher.setPosition(leftBallPusher.getPosition());
        rightBallPusher.setPosition(rightBallPusher.getPosition());
    }
}