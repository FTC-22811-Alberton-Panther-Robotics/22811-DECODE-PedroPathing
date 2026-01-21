package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

/**
 * This is a custom drivetrain implementation that integrates with the Pedro Pathing library.
 * It tells the library how to control your specific robot's mecanum wheels.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * This class, especially the inner `CustomDriveConstants`, holds the most fundamental
 * tuning values for your robot's movement.
 *
 * 1. Motor Directions: These MUST be set correctly. If your robot moves in the wrong
 *    direction (e.g., turns when it should strafe), you need to reverse the appropriate
 *    motor directions here.
 *
 * 2. Velocity Constants: `X_VELOCITY` and `Y_VELOCITY` are your robot's maximum
 *    theoretical speeds in inches per second. These are typically found using the
 *    `DriveTuning` OpMode provided by the Pedro Pathing library. Accurate values here
 *    are critical for the path follower to generate smooth and accurate paths.
 *
 * 3. Static Friction: `staticFrictionCoefficient` is a value that helps the library
 *    overcome the initial static friction of your drivetrain, preventing the robot from
 *    stalling at very low power commands. This is also found via the tuning OpMode.
 * ---------------------------------------------------------------------------------
 */
public class CustomMecanumDrive extends Drivetrain {

    /**
     * This inner class holds all the tuning constants for the drivetrain.
     * Encapsulating them here makes them easy to find and adjust.
     */
    public static class CustomDriveConstants {
        // TODO: Verify these motor names match your robot's configuration.
        public final String LEFT_FRONT_MOTOR_NAME = "leftFrontDrive";
        public final String LEFT_BACK_MOTOR_NAME = "leftBackDrive";
        public final String RIGHT_FRONT_MOTOR_NAME = "rightFrontDrive";
        public final String RIGHT_BACK_MOTOR_NAME = "rightBackDrive";

        // TODO: Set these directions correctly for your robot. Test by driving in TeleOp.
        public final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction LEFT_BACK_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
        public final DcMotor.Direction RIGHT_BACK_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

        // TODO: These values MUST be tuned using the library's DriveTuning OpMode.
        public double X_VELOCITY = 58.291667307455704; // Max X velocity (inches/sec)
        public double Y_VELOCITY = 58.291667307455704; // Max Y velocity (inches/sec)

        public final String VOLTAGE_SENSOR_NAME = "Control Hub";

        // This converts your max X and Y velocities into a vector for the library's calculations.
        private final double[] convertToPolar = Pose.cartesianToPolar(X_VELOCITY, -Y_VELOCITY);
        public final Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

        public double maxPower = 1.0;
        public double motorCachingThreshold = 0.01;
        public boolean useBrakeModeInTeleOp = false;
        public boolean useVoltageCompensation = false;
        public double nominalVoltage = 12.0;
        public double staticFrictionCoefficient = 0.1;
    }

    private final CustomDriveConstants constants;
    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    public CustomMecanumDrive(HardwareMap hardwareMap, CustomDriveConstants constants) {
        this.constants = constants;

        this.maxPowerScaling = constants.maxPower;
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.get(constants.VOLTAGE_SENSOR_NAME);

        leftFront = hardwareMap.get(DcMotorEx.class, constants.LEFT_FRONT_MOTOR_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, constants.LEFT_BACK_MOTOR_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, constants.RIGHT_BACK_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, constants.RIGHT_FRONT_MOTOR_NAME);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        updateConstants(); // Set motor directions
        setMotorsToFloat();
        breakFollowing();

        Vector copiedFrontLeftVector = constants.frontLeftVector.normalize();
        Vector oppositeVector = new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta());

        vectors = new Vector[]{
                copiedFrontLeftVector,
                oppositeVector,
                copiedFrontLeftVector,
                oppositeVector
        };
    }

    @Override
    public void updateConstants() {
        leftFront.setDirection(constants.LEFT_FRONT_MOTOR_DIRECTION);
        leftRear.setDirection(constants.LEFT_BACK_MOTOR_DIRECTION);
        rightFront.setDirection(constants.RIGHT_FRONT_MOTOR_DIRECTION);
        rightRear.setDirection(constants.RIGHT_BACK_MOTOR_DIRECTION);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        double[] wheelPowers = new double[4];
        Vector[] mecanumVectorsCopy = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = vectors[i].copy();
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double wheelPowerMax = 0;
        for (double power : wheelPowers) {
            if(Math.abs(power) > wheelPowerMax) wheelPowerMax = Math.abs(power);
        }

        if (wheelPowerMax > maxPowerScaling) {
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] = (wheelPowers[i] / wheelPowerMax) * maxPowerScaling;
            }
        }

        return wheelPowers;
    }

    @Override
    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) setMotorsToBrake();
        else setMotorsToFloat();
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) setMotorsToBrake();
        else setMotorsToFloat();
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

    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }
}
