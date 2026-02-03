package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.geometry.Pose;

/**
 * This class contains preset static field positions for the robot.
 * Using a central class for these poses allows for easy tuning and consistency
 * across different OpModes (TeleOp and Autonomous).
 * All poses are defined in inches and radians, following the coordinate system
 * conventions of the Pedro Pathing library.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Coordinate System: Ensure you understand the field coordinate system. (0,0) is
 *    in the bottom left corner of the field, on the blue alliance goal side, near the
 *    red alliance human player loading zone. All these poses MUST be measured and defined
 *    relative to that same origin for localization to work correctly.
 * 2. Pose Values: These poses must be carefully measured on a physical field. An
 *    inaccurate pose will cause the robot to miss its target during autonomous routines.
 *    Use a tuning OpMode to drive the robot to each physical location and record the
 *    X, Y, and Heading values from telemetry.
 * ---------------------------------------------------------------------------------
 */
public class FieldPosePresets {

    // TODO: Verify all of these pose values on a physical field before competition.

    // ========== BLUE ALLIANCE POSES ==========
    /** Starting position for the blue alliance, closer to the audience. */
    public static Pose BLUE_FRONT_START = new Pose(56, 9, Math.toRadians(90));
    /** Starting position for the blue alliance, closer to the back wall. */
    public static Pose BLUE_BACK_START = new Pose(23, 120, Math.toRadians(270));

    /** A general-purpose scoring position for the blue alliance. */
    public static Pose BLUE_SCORE_CLOSE_TO_GOAL = new Pose(60, 84, Math.toRadians(135));
    public static Pose BLUE_SCORE_FAR_FROM_GOAL = new Pose(54, 12, Math.toRadians(112));
    /** Pose to pick up an artifact from the front spike mark. */
    public static Pose BLUE_PICKUP_FRONT_SPIKE = new Pose(40, 36, Math.toRadians(0));
    /** Pose to pick up an artifact from the middle spike mark. */
    public static Pose BLUE_PICKUP_MIDDLE_SPIKE = new Pose(40, 60, Math.toRadians(0));
    /** Pose to pick up an artifact from the back spike mark. */
    public static Pose BLUE_PICKUP_BACK_SPIKE = new Pose(40, 84, Math.toRadians(0));

    /** A safe parking position for the blue alliance. */
    public static Pose BLUE_AUTO_PARK = new Pose(60, 60, Math.toRadians(180));
    /** A position to approach the gate before passing through. */
    public static Pose BLUE_GATE_APPROACH = new Pose(20, 72, Math.toRadians(180));
    /** The position to be at when triggering the gate. */
    public static Pose BLUE_GATE_TRIGGER = new Pose(16, 72, Math.toRadians(180));
    /** A position to read the obelisk from the blue side. */
    public static Pose BLUE_READ_OBELISK_POSE = new Pose(48, 108, Math.toRadians(90));


    // ========== RED ALLIANCE POSES ==========
    /** Starting position for the red alliance, closer to the audience. */
    public static Pose RED_FRONT_START = new Pose(88, 9, Math.toRadians(90));
    /** Starting position for the red alliance, closer to the back wall. */
    public static Pose RED_BACK_START = new Pose(121, 120, Math.toRadians(270));

    /** A general-purpose scoring position for the red alliance. */
    public static Pose RED_SCORE_CLOSE_TO_GOAL = new Pose(84, 84, Math.toRadians(45));
    public static Pose RED_SCORE_FAR_FROM_GOAL = new Pose(90, 12, Math.toRadians(68));
    /** Pose to pick up an artifact from the front spike mark. */
    public static Pose RED_PICKUP_FRONT_SPIKE = new Pose(104, 36, Math.toRadians(180));
    /** Pose to pick up an artifact from the middle spike mark. */
    public static Pose RED_PICKUP_MIDDLE_SPIKE = new Pose(104, 60, Math.toRadians(180));
    /** Pose to pick up an artifact from the back spike mark. */
    public static Pose RED_PICKUP_BACK_SPIKE = new Pose(104, 84, Math.toRadians(180));

    /** A safe parking position for the red alliance. */
    public static Pose RED_AUTO_PARK = new Pose(84, 60, Math.toRadians(0));
    /** A position to approach the gate before passing through. */
    public static Pose RED_GATE_APPROACH = new Pose(124, 72, Math.toRadians(0));
    /** The position to be at when triggering the gate. */
    public static Pose RED_GATE_TRIGGER = new Pose(128, 72, Math.toRadians(0));
    /** A position to read the obelisk from the red side. */
    public static Pose RED_READ_OBELISK_POSE = new Pose(96, 108, Math.toRadians(90));


    // ========== SHARED POSES ==========
    /** The end game parking locations. */
    public static Pose BLUE_BASE = new Pose(104.5, 33, Math.toRadians(90));
    public static Pose RED_BASE = new Pose(39.5, 33, Math.toRadians(90));

    // ========== GOAL TARGETS FOR AIMING ==========
    /** The physical center of the blue goal, used for turret and launcher aiming calculations. */
    public static Pose BLUE_GOAL_TARGET = new Pose(0, 144, 0);
    /** The physical center of the red goal, used for turret and launcher aiming calculations. */
    public static Pose RED_GOAL_TARGET = new Pose(144, 144, 0);

}
