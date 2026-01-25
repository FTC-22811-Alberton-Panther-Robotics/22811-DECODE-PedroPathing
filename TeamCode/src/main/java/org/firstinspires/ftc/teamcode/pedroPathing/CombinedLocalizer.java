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
 * --- HOW IT WORKS: FUSION AND LATENCY CORRECTION --
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
 * ---------------------------------------------------------------------------------
 * --- RELIABILITY AND DIAGNOSTICS --
 * ---------------------------------------------------------------------------------
 * This version includes several features for improved reliability and easier debugging:
 * 1. (0,0,0) Pose Rejection: The Limelight may occasionally return an invalid pose of (0,0,0).
 *    This class explicitly checks for and rejects these poses to prevent corrupting the
 *    robot's position estimate.
 * 2. isPoseReliable Flag: This boolean flag allows other classes to know if the current
 *    pose is a real, measured position or just a default placeholder, preventing the robot
 *    from making decisions based on bad data.
 * 3. Diagnostic Telemetry: The class keeps a running total of the translational and
 *    heading errors applied by the vision system. If these numbers grow very large, it
 *    indicates a problem with either the dead-wheel tracking (e.g., incorrect constants)
 *    or the vision system.
 * ---------------------------------------------------------------------------------
 */
public class CombinedLocalizer implements Localizer { 

    private final CustomPinpointLocalizer pinpoint;
    private final LimelightAprilTagLocalizer limelight;
    private final Telemetry telemetry; // This can be null

    private boolean isPoseReliable = false;
    private double totalTranslationalError = 0.0;
    private double totalHeadingError = 0.0;

    public CombinedLocalizer(CustomPinpointLocalizer pinpoint, LimelightAprilTagLocalizer limelight, Telemetry telemetry) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    public CombinedLocalizer(CustomPinpointLocalizer pinpoint, LimelightAprilTagLocalizer limelight) {
        this(pinpoint, limelight, null);
    }

    @Override
    public void update() {
        pinpoint.update();
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelight.getRobotPoseWithLatency();
        
        if (limelightPoseData.isPresent()) {
            LimelightAprilTagLocalizer.LimelightPoseData data = limelightPoseData.get();

            // Validity check to prevent corrupting the pose with an invalid (0,0,0) from Limelight
            if (data.pose.getX() == 0 && data.pose.getY() == 0 && data.pose.getHeading() == 0) {
                if (telemetry != null) telemetry.addData("Localization", "Rejected invalid (0,0,0) vision pose.");
                return; // Do not apply this invalid correction
            }

            Pose currentPinpointPose = pinpoint.getPose();
            Pose pinpointPoseAtLatency = pinpoint.getPoseAtLatency(data.latency);

            if (currentPinpointPose != null && pinpointPoseAtLatency != null) {
                Pose error = data.pose.minus(pinpointPoseAtLatency);

                totalTranslationalError += Math.hypot(error.getX(), error.getY());
                totalHeadingError += Math.abs(Math.toDegrees(error.getHeading()));

                Pose correctedPose = currentPinpointPose.plus(error);
                setPose(correctedPose);
                pinpoint.clearPoseHistory();
                if (telemetry != null) {
                    telemetry.addData("Localization", "Applying vision correction.");
                }
            }
        } else {
            if (telemetry != null) telemetry.addData("Localization", "No vision data.");
        }

        if (telemetry != null) {
            telemetry.addData("Total Translation Error", "%.2f in", totalTranslationalError);
            telemetry.addData("Total Heading Error", "%.2f deg", totalHeadingError);
        }
    }

    public boolean isPoseReliable() {
        return isPoseReliable;
    }

    public void resetHeading() {
        Pose currentPose = getPose();
        if (currentPose == null) {
            setPose(new Pose(0, 0, Math.toRadians(90)));
            isPoseReliable = false; // Explicitly mark as unreliable
        } else {
            setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));
        }
    }

    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPose(pose);
        isPoseReliable = true;
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
        isPoseReliable = true;
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
