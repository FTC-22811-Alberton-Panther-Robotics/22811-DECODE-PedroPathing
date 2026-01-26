package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Manages all interactions with the Limelight camera for AprilTag-based localization.
 * This class implements the Localizer interface but is intended to be used as a component
 * within the CombinedLocalizer, providing pose data along with its latency.
 */
public class LimelightAprilTagLocalizer implements Localizer {

    private Limelight3A limelight;
    private Telemetry telemetry;
    private final LimelightConstants constants;
    private LimelightPoseData lastPoseData; // Store the last valid pose and its latency

    public static class LimelightPoseData {
        public final Pose pose;
        public final double latency;
        public LimelightPoseData(Pose pose, double latency) {
            this.pose = pose;
            this.latency = latency;
        }
    }

    public LimelightAprilTagLocalizer(HardwareMap hardwareMap, LimelightConstants constants, Telemetry telemetry) {
        this.constants = constants;
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, LimelightConstants.hardwareMapName);
            limelight.pipelineSwitch(0);
            if (telemetry != null) telemetry.addLine("Limelight AprilTag Localizer Initialized Successfully");
        } catch (Exception e) {
            limelight = null;
            if (telemetry != null) telemetry.addLine("!!! LIMELIGHT NOT FOUND - CHECK CONFIGURATION !!!");
        }
    }
    
    public Optional<LimelightPoseData> getLatestPoseWithLatency() {
        if (limelight == null) return Optional.empty();

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return Optional.empty();
        }

        boolean hasValidGoalTag = result.getFiducialResults().stream()
                .anyMatch(tag -> constants.GOAL_TAG_IDS.contains(tag.getFiducialId()));

        if (hasValidGoalTag) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null && (botpose.getPosition().x != 0.0 || botpose.getPosition().y != 0.0)) {
                Pose ftcPose = new Pose(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw(AngleUnit.RADIANS), FTCCoordinates.INSTANCE);
                Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                double latencySeconds = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
                lastPoseData = new LimelightPoseData(pedroPose, latencySeconds);
                return Optional.of(lastPoseData);
            }
        }

        return Optional.empty();
    }

    @Override
    public void update() {
        // The logic is now handled in getLatestPoseWithLatency to be used by CombinedLocalizer
    }

    @Override
    public Pose getPose() {
        // This method is required by the interface, but for fusion, we need the latency,
        // so CombinedLocalizer will call getLatestPoseWithLatency() instead.
        return (lastPoseData != null) ? lastPoseData.pose : null;
    }

    @Override public void setPose(Pose setPose) {} 
    @Override public boolean isNAN() { return lastPoseData == null; } 
    @Override public Pose getVelocity() { return new Pose(); } 
    @Override public Vector getVelocityVector() { return new Vector(); } 
    @Override public void setStartPose(Pose setStart) {} 
    @Override public double getTotalHeading() { return (lastPoseData != null) ? lastPoseData.pose.getHeading() : 0.0; } 
    @Override public double getForwardMultiplier() { return 1.0; } 
    @Override public double getLateralMultiplier() { return 1.0; } 
    @Override public double getTurningMultiplier() { return 1.0; } 
    @Override public void resetIMU() {} 
    @Override public double getIMUHeading() { return 0; }
}
