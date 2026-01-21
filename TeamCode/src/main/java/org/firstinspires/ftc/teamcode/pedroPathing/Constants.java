package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;

/**
 * This class centralizes the creation and configuration of the Pedro Pathing Follower.
 * It defines the robot's physical constants and path constraints, and it provides
 * a single, consistent way to build a Follower instance for any OpMode.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- HOW IT WORKS ---
 * ---------------------------------------------------------------------------------
 * The `Follower` is the main engine of the pathing library. It needs to know about
 * the robot's physical properties (`FollowerConstants`), its movement limits
 * (`PathConstraints`), how its drivetrain works (`CustomMecanumDrive`), and how it
 * knows its position on the field (`Localizer`).
 * <p>
 * This class uses the `FollowerBuilder` pattern to construct a `Follower` instance.
 * Crucially, it gets the `Localizer` from our `RobotHardwareContainer`, ensuring that
 * every part of the code that needs to know the robot's position is using the same
 * single source of truth—our `CombinedLocalizer`.
 * <p>
 * The overloaded `createFollower` method is a specific workaround to support the library's
 * built-in tuning OpModes, which don't know about our `RobotHardwareContainer` structure.
 * ---------------------------------------------------------------------------------
 */
public class Constants {

    // TODO: Tune these constants for your robot's specific physical properties and desired behavior.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.11); // Robot mass at 28.9 lbs, converted to kg.

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, // Max velocity percentage
            100,  // Max acceleration
            1,    // De-acceleration ramp (don't change)
            1     // De-acceleration ramp (don't change)
    );

    /**
     * Overloaded method for creating a Follower with a specific Localizer. This is
     * specifically designed to support the library's internal Tuning OpMode without
     * requiring modifications to that file.
     */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        // If the localizer from Tuning.java is null, create a new default one on the fly.
        if (localizer == null) {
            CustomPinpointLocalizer pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointConstants());
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
