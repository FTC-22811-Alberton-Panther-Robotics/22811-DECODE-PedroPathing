package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turrethardware {
    public DcMotorEx turretMotor = null;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;


    private static final double SPIN_POWER = 1.0;


    public void init(HardwareMap hardwareMap){
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightFlywheel");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);/// FIND ACUTAL DIRICTION

        stop();
    }

    ///  ** shooter control ** \\
    public void shooterWoundUp(){
        leftFlywheel.setVelocity(2400);
        rightFlywheel.setVelocity(2400);
    }
    /// ** turret control ** \\\
    public void rightSpin(){
        turretMotor.setVelocity(100);
    }
    public void leftSpin(){
        turretMotor.setVelocity(-100);
    }

    public void stop() {
        turretMotor.setVelocity(0.0);
        leftFlywheel.setVelocity(0.0);
    }




}
