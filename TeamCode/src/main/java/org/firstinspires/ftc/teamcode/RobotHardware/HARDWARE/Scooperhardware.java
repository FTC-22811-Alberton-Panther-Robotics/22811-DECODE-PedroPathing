package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scooperhardware {

    public Servo ballScoop;

    public void init(HardwareMap hardwaremap){
        ballScoop = hardwaremap.get(Servo.class, "ballScoop");
        stop();
    }

    public void ballUp(){
        ballScoop.setPosition(1);
    }
    public void ballDown(){
        ballScoop.setPosition(1);
    }



    public void stop(){
        ballScoop.setPosition(ballScoop.getPosition());
    }








}
