package org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turrethardware {
    public DcMotorEx turretMotor = null;
    public DcMotorEx shooterMotor = null;

    private static final double SPIN_POWER = 1.0;


    public void init(HardwareMap hardwareMap){
        shooterMotor = hardwareMap.get(DcMotorEx.class,"Shooter");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);/// FIND ACUTAL DIRICTION

        stop();
    }

    ///  ** shooter control ** \\
    public void shooterWoundUp(){shooterMotor.setVelocity(100.0);
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
        shooterMotor.setVelocity(0.0);
    }




}
