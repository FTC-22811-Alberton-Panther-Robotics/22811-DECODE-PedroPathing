package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * DEPRECATED FOR TELEOP USE. This class manages the robot's mecanum drive train at a low level.
 * <p>
 * In the current robot design, all TeleOp drive commands should be sent to the
 * {@link DriverAssist} class, which uses the advanced localization and control from the
 * Pedro Pathing library's Follower.
 * <p>
 * This class is kept in the project to support very simple, time-based autonomous routines
 * (like AutoTimeDriveAndStop) that may be used as a fallback if more complex systems fail.
 * It provides direct, power-based control of the drive motors.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * Even for fallback use, these settings must be correct for the robot to move predictably.
 *
 * 1. Motor Directions: If the robot does not drive correctly (e.g., strafing makes it
 *    turn), you must reverse the direction of the appropriate motors. Test this by
 *    creating a simple OpMode that calls `drive(0.5, 0, 0)` and ensuring the robot
 *    moves straight forward.
 * 2. IMU Orientation: The `RevHubOrientationOnRobot` must match the physical orientation
 *    of the Control Hub on your robot. If it is incorrect, field-relative driving
 *    will be unpredictable. The Logo and USB facing directions are from the perspective
 *    of looking down on the robot from above.
 * ---------------------------------------------------------------------------------
 */
public class MecanumHardware {
    public DcMotor rightBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor leftFrontDrive;
    private IMU imu;

    public void init(HardwareMap hardwareMap) {

        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        imu = hardwareMap.get(IMU.class,"imu");

        // DONE: Verify these motor directions. If the robot moves incorrectly, reverse the appropriate motor(s).
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // DONE: Verify the Control Hub orientation is correct. This is critical for field-relative driving.
        RevHubOrientationOnRobot RevHubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevHubOrientation));
    }
    public void drive(double forward, double strafe, double turn) {

        double leftFrontPower = forward + strafe + turn;
        double leftBackPower = forward - strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double rightBackPower = forward + strafe - turn;

        // Normalize the wheel speeds to be between -1 and 1
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void driveFieldRelative(double forward, double strafe, double turn){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta-
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newforward = r = Math.sin(theta);
        double newstrafe = r = Math.cos(theta);

        this.drive(newforward,newstrafe, turn);
    }

    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
