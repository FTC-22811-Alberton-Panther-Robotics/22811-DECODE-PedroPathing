package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Import our new custom constants class
import org.firstinspires.ftc.teamcode.pedroPathing.CustomPinpointConstants;

public class Constants {
    // ADDED: Master switch for robot configuration
    public static final boolean IS_COMPETITION_BOT = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.35)
            .forwardZeroPowerAcceleration(-35)
            .lateralZeroPowerAcceleration(-58)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .1, 0.06))
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

        private final double[] convertToPolar = Pose.cartesianToPolar(X_VELOCITY, -Y_VELOCITY);
        public final Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

        public double maxPower = 1.0;
        public double motorCachingThreshold = 0.01;
        public boolean useBrakeModeInTeleOp = false;
        public boolean useVoltageCompensation = false;
        public double nominalVoltage = 12.0;
        public double staticFrictionCoefficient = 0.1;
    }

    public static CustomDriveConstants customDriveConstants = new CustomDriveConstants();

    // CORRECTED: Use our new, custom constants class
    public static CustomPinpointConstants localizerConstants = new CustomPinpointConstants()
            .forwardPodY(-3.13)
            .strafePodX(-6.2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    /**
     * Creates a Follower instance with the new CombinedLocalizer.
     * @param hardwareMap The hardware map from the OpMode.
     * @param localizer The fused localizer instance from the RobotHardwareContainer.
     * @return A configured Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(localizer) // Use the fused localizer
                .pathConstraints(pathConstraints)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, customDriveConstants))
                .build();
    }
}
