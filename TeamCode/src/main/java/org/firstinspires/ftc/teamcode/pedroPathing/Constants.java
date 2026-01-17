package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;

/**
 * This class centralizes the creation of the Follower object and its constants.
 */
public class Constants {

    // As per the documentation, define FollowerConstants and PathConstraints here.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.11); // Robot mass at 28.9 lbs, converted to kg.

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Overloaded method for creating a Follower with a specific Localizer. This is 
     * specifically designed to support the library's internal Tuning OpMode without
     * requiring modifications to that file.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        // If the localizer from Tuning.java is null, create a new default one on the fly.
        if (localizer == null) {
            CustomPinpointLocalizer pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointLocalizer.CustomPinpointConstants());
            LimelightAprilTagLocalizer limelight = new LimelightAprilTagLocalizer();
            limelight.init(hardwareMap);
            localizer = new CombinedLocalizer(pinpoint, limelight);
        }

        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, new CustomMecanumDrive.CustomDriveConstants()))
                .setLocalizer(localizer)
                .pathConstraints(pathConstraints)
                .build();
    }

    /**
     * Creates and configures a Follower instance for competition OpModes using the 
     * centralized localizer from the RobotHardwareContainer.
     * @param hardwareMap The hardwareMap from the OpMode.
     * @param robot The instance of the RobotHardwareContainer.
     * @return A fully initialized Follower instance.
     */
    public static Follower createFollower(HardwareMap hardwareMap, RobotHardwareContainer robot) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(new CustomMecanumDrive(hardwareMap, new CustomMecanumDrive.CustomDriveConstants()))
                .setLocalizer(robot.localizer) // Use the single source of truth
                .pathConstraints(pathConstraints)
                .build();
    }
}
