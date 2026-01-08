package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class BallTransfer {

    public Servo leftBallPusher;
    public Servo rightBallPusher;

    public void init(HardwareMap hardwaremap){
        rightBallPusher = hardwaremap.get(Servo.class, "rightPusher");
        stop();
        leftBallPusher = hardwaremap.get(Servo.class, "leftPusher");
        stop();
    }

    public void leftRun(){
        rightBallPusher.setPosition(0);
        leftBallPusher.setPosition(1);


    }

    public void leftReverse(){
        rightBallPusher.setPosition(-1);
        leftBallPusher.setPosition(0);


    }

    public void stopSpin() {
        rightBallPusher.setPosition(0);
        leftBallPusher.setPosition(0);

    }
    public void rightRun() {
        rightBallPusher.setPosition(0);
        leftBallPusher.setPosition(-1);


    }
    public void rightReverse(){
        rightBallPusher.setPosition(1);
        leftBallPusher.setPosition(0);

    }



    public void stop(){
        leftBallPusher.setPosition(leftBallPusher.getPosition());
        rightBallPusher.setPosition(rightBallPusher.getPosition());
    }
}


