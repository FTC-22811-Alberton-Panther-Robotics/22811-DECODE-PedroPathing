package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * A static class to hold game state information that needs to persist
 * between OpModes (e.g., from Autonomous to TeleOp).
 */
public class GameState {

    // Using enums for clarity and type safety.
    public enum Alliance { BLUE, RED }
    public enum ObeliskPattern { UNKNOWN, PATTERN_GPP, PATTERN_PGP, PATTERN_PPG }

    // The `volatile` keyword ensures that changes are immediately visible across OpModes.
    public static volatile Alliance alliance = Alliance.BLUE;
    public static volatile ObeliskPattern obeliskPattern = ObeliskPattern.UNKNOWN;
    public static volatile Pose currentPose = null; // To store the robot's pose at the end of auto

    // The definitive mapping from AprilTag ID to Obelisk Pattern
    public static final int PATTERN_GPP_ID = 21;
    public static final int PATTERN_PGP_ID = 22;
    public static final int PATTERN_PPG_ID = 23;

    /**
     * Sets the obelisk pattern based on a detected AprilTag ID.
     * This is the single source of truth for interpreting the obelisk randomization.
     * @param aprilTagId The ID of the AprilTag detected on the obelisk.
     */
    public static void setPatternFromAprilTagId(int aprilTagId) {
        if (aprilTagId == PATTERN_GPP_ID) {
            obeliskPattern = ObeliskPattern.PATTERN_GPP;
        } else if (aprilTagId == PATTERN_PGP_ID) {
            obeliskPattern = ObeliskPattern.PATTERN_PGP;
        } else if (aprilTagId == PATTERN_PPG_ID) {
            obeliskPattern = ObeliskPattern.PATTERN_PPG;
        } else {
            obeliskPattern = ObeliskPattern.UNKNOWN;
        }
    }
}
