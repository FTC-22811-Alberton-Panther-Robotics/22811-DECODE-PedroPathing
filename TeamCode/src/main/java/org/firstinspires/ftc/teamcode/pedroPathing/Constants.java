package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class centralizes the creation of the Follower object and all its dependencies.
 * By creating the entire hardware and localization stack here, we ensure that any
 * OpMode (Tuning, TeleOp, or Autonomous) gets a fully initialized Follower, resolving
 * potential NullPointerExceptions.
 */
public class Constants {

    /**
     * Creates and configures a Follower instance with a custom-fused localization stack.
     * @param hardwareMap The hardwareMap from the OpMode.
     * @param telemetry The telemetry object from the OpMode, used for logging.
     * @return A fully initialized Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Telemetry telemetry) {

        // 1. Create the constants for the dead-wheel localizer
        CustomPinpointConstants pinpointConstants = new CustomPinpointConstants();

        // 2. Create the instances of our two underlying localizers
        CustomPinpointLocalizer pinpointLocalizer = new CustomPinpointLocalizer(hardwareMap, pinpointConstants);
        LimelightAprilTagLocalizer limelightLocalizer = new LimelightAprilTagLocalizer();
        limelightLocalizer.init(hardwareMap, telemetry);

        // 3. Create the CombinedLocalizer that fuses them together
        CombinedLocalizer combinedLocalizer = new CombinedLocalizer(pinpointLocalizer, limelightLocalizer, telemetry);

        // 4. Build the Follower, passing in our custom drivetrain and the fused localizer
        return new FollowerBuilder(hardwareMap)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, new CustomMecanumDrive.CustomDriveConstants()))
                .setLocalizer(combinedLocalizer)
                .build();
    }
}
