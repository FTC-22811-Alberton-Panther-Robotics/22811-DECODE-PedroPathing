package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private final LimelightAprilTagLocalizer limelightLocalizer;
    private final Limelight3A limelightDevice; // The single, physical Limelight object.
    private final Telemetry telemetry; // This can be null

    // --- Constants for Fusion and Outlier Rejection ---
    // The maximum allowed difference between odometry and vision poses to be considered valid.
    private static final double MAX_ALLOWED_DEVIATION_INCHES = 6.0;
    private static final double MAX_ALLOWED_HEADING_DEVIATION_RADIANS = Math.toRadians(10);

    // The weight given to the vision measurement when fusing. 0.0 = 100% odometry, 1.0 = 100% vision.
    // A small value like 0.1 provides smooth correction.
    private static final double FUSION_ALPHA = 0.1;

    private boolean isPoseReliable = false;
    private double totalCorrectionX = 0.0, totalCorrectionY = 0.0, totalCorrectionHeading = 0.0;

    public CombinedLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // --- HARDWARE FIX: Single Source of Truth for Limelight ---
        // Use a temporary variable to handle the hardware mapping. This allows the final
        // limelightDevice field to be assigned exactly once, satisfying the `final` keyword rule.
        Limelight3A tempLimelight;
        try {
            tempLimelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            // If the Limelight is not found, the device will be null.
            // All dependent classes have null-checks to handle this.
            tempLimelight = null;
        }
        this.limelightDevice = tempLimelight;

        // 2. Create the dead-wheel localizer.
        this.pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointConstants());

        // 3. Create the vision localizer and pass it the single Limelight device instance.
        this.limelightLocalizer = new LimelightAprilTagLocalizer(this.limelightDevice, new LimelightConstants(), telemetry);
    }

    /**
     * Returns the single instance of the Limelight device so it can be shared with other subsystems
     * like the LimelightBallDetector without creating a hardware conflict.
     */
    public Limelight3A getLimelight() {
        return limelightDevice;
    }

    @Override
    public void update() {
        pinpoint.update();
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelightLocalizer.getLatestPoseWithLatency();

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
                // No need to call setPose here, which would incorrectly set isPoseReliable.
                // Instead, call the pinpoint's setPose directly.
                pinpoint.setPose(fusedPose);
                isPoseReliable = true;

                // After a successful correction, clear the pinpoint's history.
                pinpoint.clearPoseHistory();

                if (telemetry != null) {
                    telemetry.addData("Localization", "-> APPLYING VISION CORRECTION <- ");
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
        } else {
            setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(90)));
        }
        isPoseReliable = false; // Explicitly mark as unreliable after any reset
    }

    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPose(pose);
        isPoseReliable = false; // Setting a pose should require re-validation from vision
    }

    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
        isPoseReliable = true; // Setting a start pose should assume the pose is reliable since it's a preset position or remembered from auto.
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
