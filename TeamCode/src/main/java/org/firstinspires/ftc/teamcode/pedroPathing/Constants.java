package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This class centralizes the creation and configuration of the Pedro Pathing Follower.
 * It follows the design pattern from the library's documentation, defining all constants
 * and using a builder to construct the Follower.
 */
public class Constants {

    // TODO: Run all tuners and update these values. See the Pedro Pathing documentation for guides.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.11) // Robot mass in kilograms
            //.forwardZeroPowerAcceleration(-25.0) // Tuned with ForwardZeroPowerAccelerationTuner
            //.lateralZeroPowerAcceleration(-67.0) // Tuned with LateralZeroPowerAccelerationTuner
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015)) // Tuned with TranslationalPIDTuner
            //.headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01)) // Tuned with HeadingPIDTuner
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015)) // Tuned with DrivePIDTuner
            //.centripetalScaling(0.0005); // Tuned with CentripetalTuner
            ;
    // Constants for our custom dead-wheel localizer
    public static CustomPinpointConstants pinpointConstants = new CustomPinpointConstants();

    // Constants for our custom Limelight localizer
    public static LimelightConstants limelightConstants = new LimelightConstants();

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, // Max velocity percentage
            100,  // Max acceleration
            1,    // De-acceleration ramp (don't change)
            1     // De-acceleration ramp (don't change)
    );
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.58)
            .yVelocity(49.88);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.5)
            .strafePodX(0.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    /**
     * Creates a new Follower instance with a complete, fused localization system.
     *
     * @param hardwareMap The hardwareMap from the OpMode.
     * @return A fully initialized Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create the CombinedLocalizer which will be the single source of truth
        //CombinedLocalizer combinedLocalizer = new CombinedLocalizer(hardwareMap, telemetry);
        // Use the FollowerBuilder to construct the follower with our custom components
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
//                .setLocalizer(combinedLocalizer) // Inject our fused localizer
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
