package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class CustomMecanumDrive extends Drivetrain {

    private final DcMotor leftFront, leftRear, rightRear, rightFront;
    private final Constants.CustomDriveConstants constants;
    private final VoltageSensor voltageSensor;

    public CustomMecanumDrive(HardwareMap hardwareMap, Constants.CustomDriveConstants constants) {
        this.constants = constants;

        leftFront = hardwareMap.get(DcMotor.class, constants.LEFT_FRONT_MOTOR_NAME);
        leftRear = hardwareMap.get(DcMotor.class, constants.LEFT_BACK_MOTOR_NAME);
        rightRear = hardwareMap.get(DcMotor.class, constants.RIGHT_BACK_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotor.class, constants.RIGHT_FRONT_MOTOR_NAME);

        leftFront.setDirection(constants.LEFT_FRONT_MOTOR_DIRECTION);
        leftRear.setDirection(constants.LEFT_BACK_MOTOR_DIRECTION);
        rightFront.setDirection(constants.RIGHT_FRONT_MOTOR_DIRECTION);
        rightRear.setDirection(constants.RIGHT_BACK_MOTOR_DIRECTION);

        voltageSensor = hardwareMap.voltageSensor.get(constants.VOLTAGE_SENSOR_NAME);

        breakFollowing();
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        Vector totalTranslational = pathingPower.plus(correctivePower);

        // Swapped these to fix forward/strafe mix-up
        double fieldForward = totalTranslational.getXComponent();
        double fieldStrafe = totalTranslational.getYComponent();

        double robotForward = fieldForward * Math.cos(-robotHeading) - fieldStrafe * Math.sin(-robotHeading);
        double robotStrafe = fieldForward * Math.sin(-robotHeading) + fieldStrafe * Math.cos(-robotHeading);

        double turn = headingPower.getXComponent();

        // Corrected Mecanum Drive Formulas
        double[] motorPowers = new double[4];
        motorPowers[0] = robotForward + robotStrafe + turn; // Left Front
        motorPowers[1] = robotForward - robotStrafe + turn; // Left Rear
        motorPowers[2] = robotForward - robotStrafe - turn; // Right Front
        motorPowers[3] = robotForward + robotStrafe - turn; // Right Rear
        return motorPowers;
    }

    @Override
    public void runDrive(double[] drivePowers) {
        double max = 1.0;
        for (double power : drivePowers) {
            if (Math.abs(power) > max) {
                max = Math.abs(power);
            }
        }
        if (max > 1) {
            for (int i = 0; i < drivePowers.length; i++) {
                drivePowers[i] /= max;
            }
        }

        leftFront.setPower(drivePowers[0]);
        leftRear.setPower(drivePowers[1]);
        rightFront.setPower(drivePowers[2]);
        rightRear.setPower(drivePowers[3]);
    }

    @Override
    public void updateConstants() {
        // This method is called by the Follower to update the drivetrain's constants.
        // If you have any constants that can change during runtime, you can update them here.
        // For now, we'll leave this empty as our constants are final.
    }

    @Override
    public void breakFollowing() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void startTeleopDrive() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior behavior = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

    @Override
    public double xVelocity() {
        return constants.X_VELOCITY;
    }

    @Override
    public double yVelocity() {
        return constants.Y_VELOCITY;
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.X_VELOCITY = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        constants.Y_VELOCITY = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return String.format("LF: %.2f, LR: %.2f, RF: %.2f, RR: %.2f",
                leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower());
    }
}
