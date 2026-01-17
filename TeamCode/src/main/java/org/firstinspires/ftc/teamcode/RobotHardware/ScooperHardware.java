package org.firstinspires.ftc.teamcode.RobotHardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScooperHardware {

    public Servo ballScoop;

    public void init(HardwareMap hardwaremap){
        ballScoop = hardwaremap.get(Servo.class, "ballScoop");
        stop();
    }

    public void ballUp(){
        ballScoop.setPosition(1);
    }
    public void ballDown(){
        ballScoop.setPosition(0);
    }



    public void stop(){
        ballScoop.setPosition(ballScoop.getPosition());
    }








}
