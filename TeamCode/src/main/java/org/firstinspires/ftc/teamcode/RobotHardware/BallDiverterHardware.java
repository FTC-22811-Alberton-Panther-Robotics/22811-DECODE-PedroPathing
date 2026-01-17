package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BallDiverterHardware {

    public Servo ballDiverterServo;

    public void init(HardwareMap hardwaremap){
        ballDiverterServo = hardwaremap.get(Servo.class, "Diverter");
    stop();
    }

    public void greenBall(){
        ballDiverterServo.setPosition(0);


    }

    public void purpleBall(){
    ballDiverterServo.setPosition(1);

    }

    public void midStow(){
    ballDiverterServo.setPosition(.5);

    }



    public void stop(){
        ballDiverterServo.setPosition(ballDiverterServo.getPosition());
    }



}


