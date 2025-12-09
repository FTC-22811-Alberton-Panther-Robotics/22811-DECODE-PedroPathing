package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * A persistent class to store and share information between OpModes (Autonomous and TeleOp).
 * This class uses static fields, so the data will remain for the entire duration the app is running.
 */
public class GameState {

    // --- Shared State Enums ---
    public enum Alliance {
        BLUE,
        RED,
        UNKNOWN
    }

    // --- Static Fields for Persistent State ---
    // These variables will be set in Auto and read in TeleOp.
    public static Alliance alliance = Alliance.UNKNOWN;
    public static Pose currentPose = null;

    // Private constructor to prevent instantiation, as this is a static utility class.
    private GameState() {}
}
