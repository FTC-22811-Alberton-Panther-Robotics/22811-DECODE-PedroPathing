package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.35)
    .forwardZeroPowerAcceleration(-35)
    .lateralZeroPowerAcceleration(-58)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
    .headingPIDFCoefficients(new PIDFCoefficients(4, 0.03, 1, 0.06))
    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0.6, 0.03));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static class CustomDriveConstants {
        public final String LEFT_FRONT_MOTOR_NAME = "leftFrontDrive";
        public final String LEFT_BACK_MOTOR_NAME = "leftBackDrive";
        public final String RIGHT_FRONT_MOTOR_NAME = "rightFrontDrive";
        public final String RIGHT_BACK_MOTOR_NAME = "rightBackDrive";

        public final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction LEFT_BACK_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
        public final DcMotor.Direction RIGHT_BACK_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

        public double X_VELOCITY = 60.4577684779282;
        public double Y_VELOCITY = 48.871302326833174; // Set to X initially, tune if needed

        public final String VOLTAGE_SENSOR_NAME = "Control Hub";

        // This is the crucial vector needed for the library's calculations
        private final double[] convertToPolar = Pose.cartesianToPolar(X_VELOCITY, -Y_VELOCITY);
        public final Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

        // Default values from the library, can be tuned if needed
        public double maxPower = 1.0;
        public double motorCachingThreshold = 0.01;
        public boolean useBrakeModeInTeleOp = false;
        public boolean useVoltageCompensation = false;
        public double nominalVoltage = 12.0;
        public double staticFrictionCoefficient = 0.1;
    }

    public static CustomDriveConstants customDriveConstants = new CustomDriveConstants();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, customDriveConstants))
                .build();
    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-1.75)
            .strafePodX(6.625)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
