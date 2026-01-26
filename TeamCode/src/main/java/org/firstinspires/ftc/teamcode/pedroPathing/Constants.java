package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This class centralizes the creation and configuration of the Pedro Pathing Follower.
 * It follows the design pattern from the library's documentation, defining all constants
 * and using a builder to construct the Follower.
 */
public class Constants {

    // TODO: Run all tuners and update these values. See the Pedro Pathing documentation for guides.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.11) // Robot mass in kilograms
            .forwardZeroPowerAcceleration(-25.0) // Tuned with ForwardZeroPowerAccelerationTuner
            .lateralZeroPowerAcceleration(-67.0) // Tuned with LateralZeroPowerAccelerationTuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.015)) // Tuned with TranslationalPIDTuner
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0, 0.01)) // Tuned with HeadingPIDTuner
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.00035, 0.6, 0.015)) // Tuned with DrivePIDTuner
            .centripetalScaling(0.0005); // Tuned with CentripetalTuner

    // Constants for our custom Mecanum drive
    public static CustomMecanumDrive.CustomDriveConstants driveConstants = new CustomMecanumDrive.CustomDriveConstants();

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

    /**
     * Creates a new Follower instance with a complete, fused localization system.
     *
     * @param hardwareMap The hardwareMap from the OpMode.
     * @return A fully initialized Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create the CombinedLocalizer which will be the single source of truth
        CombinedLocalizer combinedLocalizer = new CombinedLocalizer(hardwareMap, telemetry);

        // Use the FollowerBuilder to construct the follower with our custom components
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, driveConstants))
                .setLocalizer(combinedLocalizer) // Inject our fused localizer
                .pathConstraints(pathConstraints)
                .build();
    }
}
