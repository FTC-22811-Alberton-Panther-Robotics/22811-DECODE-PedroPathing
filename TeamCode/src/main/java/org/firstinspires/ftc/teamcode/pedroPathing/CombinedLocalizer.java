package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Optional;

/**
 * A fused localizer that orchestrates corrections between a dead-wheel localizer
 * (CustomPinpointLocalizer) and a vision localizer (LimelightAprilTagLocalizer).
 * This class implements the Localizer interface and is the single source of truth for the
 * robot's position on the field.
 */
public class CombinedLocalizer implements Localizer {

    private final CustomPinpointLocalizer pinpoint;
    private final LimelightAprilTagLocalizer limelight;
    private final Telemetry telemetry; // This can be null

    // --- Constants for Fusion and Outlier Rejection ---
    // The maximum allowed difference between odometry and vision poses to be considered valid.
    private static final double MAX_ALLOWED_DEVIATION_INCHES = 6.0;
    private static final double MAX_ALLOWED_HEADING_DEVIATION_RADIANS = Math.toRadians(10);

    // The weight given to the vision measurement when fusing. 0.0 = 100% odometry, 1.0 = 100% vision.
    // A small value like 0.1 provides smooth correction.
    private static final double FUSION_ALPHA = 0.1;
    // ----------------------------------------------------

    private boolean isPoseReliable = false;

    // --- New members for tracking total correction ---
    private double totalCorrectionX = 0.0;
    private double totalCorrectionY = 0.0;
    private double totalCorrectionHeading = 0.0;
    // ----------------------------------------------------

    public CombinedLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointConstants());
        this.limelight = new LimelightAprilTagLocalizer(hardwareMap, new LimelightConstants(), telemetry);
        this.telemetry = telemetry;
    }

    @Override
    public void update() {
        // First, always update the primary dead-wheel localizer to populate its pose history.
        pinpoint.update();

        // Attempt to get a pose from the vision system.
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelight.getLatestPoseWithLatency();

        // If the vision system has a valid pose, attempt to fuse it.
        if (limelightPoseData.isPresent()) {
            LimelightAprilTagLocalizer.LimelightPoseData visionData = limelightPoseData.get();
            Pose visionPose = visionData.pose;
            double visionLatency = visionData.latency;

            // Get the historical odometry pose at the moment the Limelight captured its image.
            Pose historicalOdomPose = pinpoint.getPoseAtLatency(visionLatency);

            // This is the crucial step: Compare the vision pose to our HISTORICAL odometry pose.
            double deviation = historicalOdomPose.distanceFrom(visionPose);
            double headingDeviation = Math.abs(AngleUnit.normalizeRadians(historicalOdomPose.getHeading() - visionPose.getHeading()));

            if (telemetry != null) {
                telemetry.addData("LL Latency", "%.1f ms", visionLatency * 1000);
                telemetry.addData("LL Deviation", "%.2f in, %.1f deg", deviation, Math.toDegrees(headingDeviation));
            }

            // --- Outlier Rejection ---
            if (deviation < MAX_ALLOWED_DEVIATION_INCHES && headingDeviation < MAX_ALLOWED_HEADING_DEVIATION_RADIANS) {
                // The vision data is trustworthy. Let's fuse it.

                // Calculate the positional error between the vision pose and where we thought we were in the past.
                Pose error = visionPose.minus(historicalOdomPose);

                // --- Calculate the amount of correction to apply for this frame ---
                double correctionX = error.getX() * FUSION_ALPHA;
                double correctionY = error.getY() * FUSION_ALPHA;
                double correctionHeading = error.getHeading() * FUSION_ALPHA;

                // --- Update total accumulated corrections ---
                totalCorrectionX += correctionX;
                totalCorrectionY += correctionY;
                totalCorrectionHeading += correctionHeading;

                // --- Smoothed Fusion ---
                // Apply a fraction of this historical error to the CURRENT odometry pose.
                Pose currentOdomPose = pinpoint.getPose();
                double fusedX = currentOdomPose.getX() + correctionX;
                double fusedY = currentOdomPose.getY() + correctionY;
                double fusedHeading = AngleUnit.normalizeRadians(currentOdomPose.getHeading() + correctionHeading);
                Pose fusedPose = new Pose(fusedX, fusedY, fusedHeading);

                // Apply the newly corrected pose back to the pinpoint localizer
                setPose(fusedPose);

                // After a successful correction, clear the pinpoint's history.
                pinpoint.clearPoseHistory();

                if (telemetry != null) {
                    telemetry.addData("Localization", "-> APPLYING VISION CORRECTION <-");
                }
            } else {
                // The deviation is too large. Ignore this Limelight frame as an outlier.
                if (telemetry != null) {
                    telemetry.addData("Localization", "Vision outlier REJECTED");
                }
            }
        } else {
            if (telemetry != null) telemetry.addData("Localization", "No vision data");
        }

        // If we haven't gotten a vision update yet, but the dead wheels are giving us
        // valid numbers, then we can consider the pose reliable for driving.
        if (!isPoseReliable && !pinpoint.isNAN()) {
            isPoseReliable = true;
        }

        if (telemetry != null) {
            Pose currentPose = getPose();
            if(currentPose != null) {
                telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, H: %.1f", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            }
            // Add the accumulated correction telemetry
            telemetry.addData("Total Correction", "X: %.2f, Y: %.2f, H: %.1f deg", totalCorrectionX, totalCorrectionY, Math.toDegrees(totalCorrectionHeading));
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
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
        isPoseReliable = true;
    }

    // All other methods from the Localizer interface simply delegate to the pinpoint localizer.
    @Override public Pose getVelocity() { return pinpoint.getVelocity(); }
    @Override public Vector getVelocityVector() { return pinpoint.getVelocityVector(); }
    @Override public double getTotalHeading() { return pinpoint.getTotalHeading(); }
    @Override public double getForwardMultiplier() { return pinpoint.getForwardMultiplier(); }
    @Override public double getLateralMultiplier() { return pinpoint.getLateralMultiplier(); }
    @Override public double getTurningMultiplier() { return pinpoint.getTurningMultiplier(); }
    @Override public void resetIMU() throws InterruptedException { pinpoint.resetIMU(); }
    @Override public double getIMUHeading() { return pinpoint.getIMUHeading(); }
    @Override public boolean isNAN() { return pinpoint.isNAN(); }
}
