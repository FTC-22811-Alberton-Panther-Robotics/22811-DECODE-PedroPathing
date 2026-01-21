package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * A static class to hold game state information that needs to persist between OpModes
 * (e.g., from Autonomous to TeleOp).
 * <p>
 * By declaring variables as `public static volatile`, we ensure that any changes made
 * to them in one OpMode (like setting the `currentPose` at the end of Autonomous) are
 * immediately visible and accessible to the next OpMode (like TeleOp).
 * <p>
 * This is a simple but effective way to pass data without writing to files.
 */
public class GameState {

    /** Represents the team's alliance color. */
    public enum Alliance { BLUE, RED, UNKNOWN }

    /** Represents the randomized pattern on the Obelisk, determined by an AprilTag. */
    public enum ObeliskPattern { UNKNOWN, PATTERN_GPP, PATTERN_PGP, PATTERN_PPG }

    // The `volatile` keyword ensures that changes are immediately visible across threads/OpModes.
    public static volatile Alliance alliance = Alliance.UNKNOWN;
    public static volatile ObeliskPattern obeliskPattern = ObeliskPattern.UNKNOWN;
    public static volatile Pose currentPose = null;

    // TODO: Verify these AprilTag IDs match the official game manual for the season.
    public static final int PATTERN_GPP_ID = 21;
    public static final int PATTERN_PGP_ID = 22;
    public static final int PATTERN_PPG_ID = 23;

    /**
     * Sets the obelisk pattern based on a detected AprilTag ID.
     * This is the single source of truth for interpreting the obelisk randomization.
     * @param aprilTagId The ID of the AprilTag detected on the obelisk.
     */
    public static void setPatternFromAprilTagId(int aprilTagId) {
        switch (aprilTagId) {
            case PATTERN_GPP_ID:
                obeliskPattern = ObeliskPattern.PATTERN_GPP;
                break;
            case PATTERN_PGP_ID:
                obeliskPattern = ObeliskPattern.PATTERN_PGP;
                break;
            case PATTERN_PPG_ID:
                obeliskPattern = ObeliskPattern.PATTERN_PPG;
                break;
            default:
                obeliskPattern = ObeliskPattern.UNKNOWN;
                break;
        }
    }
}
