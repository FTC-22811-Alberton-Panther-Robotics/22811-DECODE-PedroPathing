package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Pedro Pathing imports
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.FTCCoordinates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Manages all interactions with the Limelight 3A camera for the DECODE game season.
 * This class handles initialization, AprilTag detection, and provides methods
 * to get detected tag IDs and the robot's field-relative pose with latency.
 */
public class LimelightAprilTagLocalizer {

    private Limelight3A limelight;
    private Telemetry telemetry;

    // AprilTag IDs for the goals, used for localization.
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24);

    // --- New variables for enhanced telemetry ---
    private int successfulPoseCalculations = 0;
    private List<Integer> lastSeenTagIds = new ArrayList<>();

    /**
     * A simple data class to hold both the calculated pose and the latency of the reading.
     */
    public static class LimelightPoseData {
        public final Pose pose;
        public final double latency; // in seconds

        public LimelightPoseData(Pose pose, double latency) {
            this.pose = pose;
            this.latency = latency;
        }
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            if (telemetry != null) telemetry.addLine("Limelight Initialized Successfully");
        } catch (Exception e) {
            limelight = null;
            if (telemetry != null) telemetry.addLine("!!! LIMELIGHT NOT FOUND - CHECK CONFIGURATION !!!");
        }
    }

    // Overloaded method for initialization without telemetry
    public void init(HardwareMap hardwareMap) {
        this.init(hardwareMap, null);
    }

    /**
     * Gets the ID of the first valid AprilTag the Limelight detects.
     * This is useful for reading the randomization pattern.
     * @return An Optional containing the ID, or an empty Optional if no tag is found.
     */
    public Optional<Integer> getDetectedTagId() {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (!result.getFiducialResults().isEmpty()) {
                int detectedId = result.getFiducialResults().get(0).getFiducialId();
                if (telemetry != null) telemetry.addData("Limelight Detected ID", detectedId);
                return Optional.of(detectedId);
            }
        }

        if (telemetry != null) telemetry.addData("Limelight Detected ID", "None");
        return Optional.empty();
    }

    /**
     * Gets the robot's field-relative pose and latency, and updates detailed telemetry.
     * @return An Optional containing the robot's pose and latency, or an empty Optional otherwise.
     */
    public Optional<LimelightPoseData> getRobotPoseWithLatency() {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();

        // --- Enhanced Telemetry Logic ---
        if (result == null || !result.isValid()) {
            if (telemetry != null) telemetry.addLine("Limelight Result: INVALID or NULL");
            lastSeenTagIds.clear();
        } else {
            // Record all tags seen in this frame.
            lastSeenTagIds = result.getFiducialResults().stream()
                                    .map(LLResultTypes.FiducialResult::getFiducialId)
                                    .collect(Collectors.toList());

            boolean hasValidTag = false;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (GOAL_TAG_IDS.contains(tag.getFiducialId())) {
                    hasValidTag = true;
                    break;
                }
            }

            if (hasValidTag) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null && (botpose.getPosition().x != 0.0 || botpose.getPosition().y != 0.0)) {
                    successfulPoseCalculations++; // Increment our success counter.

                    Position position = botpose.getPosition();
                    YawPitchRollAngles orientation = botpose.getOrientation();
                    Pose ftcPose = new Pose(
                            position.x,
                            position.y,
                            orientation.getYaw(AngleUnit.RADIANS),
                            FTCCoordinates.INSTANCE
                    );
                    Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                    double latencySeconds = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
                    
                    if (telemetry != null) {
                        telemetry.addData("Limelight Pose (X, Y, H)", "%.2f, %.2f, %.1f",
                            pedroPose.getX(), pedroPose.getY(), Math.toDegrees(pedroPose.getHeading()));
                        telemetry.addData("Limelight Latency (s)", "%.3f", latencySeconds);
                    }
                    
                    updateDebugTelemetry();
                    return Optional.of(new LimelightPoseData(pedroPose, latencySeconds));
                }
            }
        }

        if (telemetry != null) telemetry.addLine("Limelight Pose: None (No valid GOAL tag found or bad botpose)");
        updateDebugTelemetry();
        return Optional.empty();
    }

    /**
     * Helper method to send detailed debug information to telemetry every loop.
     */
    private void updateDebugTelemetry() {
        if (telemetry != null) {
            telemetry.addData("LL Valid Poses Count", successfulPoseCalculations);
            telemetry.addData("LL Last Seen IDs", lastSeenTagIds.toString());
        }
    }
}
