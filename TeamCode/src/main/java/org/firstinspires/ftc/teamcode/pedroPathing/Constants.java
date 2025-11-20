package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.25);
    // These are just examples, you will need to tune these for your robot
    // .forwardZeroPowerAcceleration(-25.9)
    // .lateralZeroPowerAcceleration(-67.3)
    // .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015))
    // .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static class CustomDriveConstants {
        // Motor names in hardware configuration
        public final String LEFT_FRONT_MOTOR_NAME = "leftFrontDrive";
        public final String LEFT_BACK_MOTOR_NAME = "leftBackDrive";
        public final String RIGHT_FRONT_MOTOR_NAME = "rightFrontDrive";
        public final String RIGHT_BACK_MOTOR_NAME = "rightBackDrive";

        // Motor directions that work for your robot
        public final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction LEFT_BACK_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
        public final DcMotor.Direction RIGHT_BACK_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

        // FTC Dashboard measurement from the Follower and Drive Characterization OpModes
        public double X_VELOCITY = 58.291667307455704;
        public double Y_VELOCITY = 58.291667307455704; // Set to X initially, tune if needed

        // Hardware map name for the voltage sensor. "Control Hub" is the default.
        public final String VOLTAGE_SENSOR_NAME = "Control Hub";
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
            .forwardPodY(-8.5)
            .strafePodX(2.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
