package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Optional;

/**
 * A fused localizer that orchestrates corrections between a dead-wheel localizer
 * (CustomPinpointLocalizer) and a vision localizer (LimelightAprilTagLocalizer).
 * This class is the single source of truth for the robot's position on the field.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- HOW IT WORKS ---
 * ---------------------------------------------------------------------------------
 * The challenge with fusing two localization systems is that they have different latencies.
 * Dead-wheel encoders provide very fast but slightly inaccurate updates (due to wheel slip),
 * while the Limelight provides very accurate but slow updates (due to camera exposure,
 * network time, and image processing).
 * <p>
 * This class solves this problem with the following logic:
 * 1. It continuously updates the dead-wheel localizer (`pinpoint.update()`).
 * 2. It also continuously asks the Limelight for a new pose (`limelight.getRobotPoseWithLatency()`).
 * 3. When the Limelight returns a valid pose, it also tells us its `latency`—how old the data is.
 * 4. We then ask our dead-wheel localizer where it *thought* the robot was at that exact moment
 *    in the past (`pinpoint.getPoseAtLatency(latency)`).
 * 5. The difference between the Limelight's accurate pose and the dead-wheel's historical pose
 *    is calculated. This difference is the `error`.
 * 6. This `error` is then applied to the *current* dead-wheel pose, instantly correcting it.
 * 7. The dead-wheel localizer's history is then cleared to prevent applying the same correction twice.
 * <p>
 * By delegating all other method calls (like `getPose()`) to the underlying dead-wheel
 * localizer, the rest of the robot code can get a smooth, fast, and frequently-corrected
 * position estimate without needing to know about the complexity of the fusion logic.
 * ---------------------------------------------------------------------------------
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
