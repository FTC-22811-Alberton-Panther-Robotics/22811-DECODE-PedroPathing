package org.firstinspires.ftc.teamcode.pedroPathing;

import java.util.Arrays;
import java.util.List;

/**
 * This class holds all the configuration constants for the LimelightAprilTagLocalizer.
 * Following the library's design pattern, this keeps all tunable values in one place.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Goal Tag IDs: This list MUST contain the specific AprilTag IDs that are on the
 *    scoring goals for the current season. The localizer will only use these tags
 *    for position correction.
 * 2. Hardware Map Name: Ensure the `hardwareMapName` matches the name you gave the
 *    Limelight in your robot's configuration file.
 * ---------------------------------------------------------------------------------
 */
public class LimelightConstants {

    // TODO: Verify these are the correct AprilTag IDs for the goal targets this season.
    public static List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24);

    public static String hardwareMapName = "limelight";

}
