package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

/**
 * A fused localizer that orchestrates corrections between a dead-wheel localizer
 * (PinpointHardware) and a vision localizer (LimelightAprilTagLocalizer).
 */
public class CombinedLocalizer implements Localizer { 

    private final CustomPinpointLocalizer pinpoint;
    private final LimelightAprilTagLocalizer limelight;
    private final Telemetry telemetry; // This can be null

    public CombinedLocalizer(CustomPinpointLocalizer pinpoint, LimelightAprilTagLocalizer limelight, Telemetry telemetry) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    // Overloaded constructor for when telemetry is not available
    public CombinedLocalizer(CustomPinpointLocalizer pinpoint, LimelightAprilTagLocalizer limelight) {
        this(pinpoint, limelight, null);
    }

    @Override
    public void update() {
        pinpoint.update();
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelight.getRobotPoseWithLatency();
        
        // CORRECTED: This is the definitive fix for the NullPointerException.
        // We must ensure the pinpoint localizer has a valid pose before attempting a correction.
        if (limelightPoseData.isPresent()) {
            Pose currentPinpointPose = pinpoint.getPose();
            Pose pinpointPoseAtLatency = pinpoint.getPoseAtLatency(limelightPoseData.get().latency);

            // Only apply correction if both the current and historical pinpoint poses are valid.
            if (currentPinpointPose != null && pinpointPoseAtLatency != null) {
                Pose error = limelightPoseData.get().pose.minus(pinpointPoseAtLatency);
                Pose correctedPose = currentPinpointPose.plus(error);
                pinpoint.setPose(correctedPose);
                pinpoint.clearPoseHistory();
                if (telemetry != null) {
                    telemetry.addData("Localization", "Applying vision correction.");
                }
            }
        }
    }

    // ======================================================================================
    // All other methods simply delegate to the underlying dead-wheel localizer.
    // ======================================================================================

    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPose(pose);
    }

    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
    }

    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return pinpoint.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return pinpoint.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return pinpoint.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        pinpoint.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return pinpoint.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }
}
