package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private boolean isPoseReliable = false;
    private double totalTranslationalError = 0.0;
    private double totalHeadingError = 0.0;

    public CombinedLocalizer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointConstants());
        this.limelight = new LimelightAprilTagLocalizer(hardwareMap, new LimelightConstants(), telemetry);
        this.telemetry = telemetry;
    }

    @Override
    public void update() {
        // First, update the underlying localizers.
        pinpoint.update();
        
        // Attempt to get a pose from the vision system.
        Optional<LimelightAprilTagLocalizer.LimelightPoseData> limelightPoseData = limelight.getLatestPoseWithLatency();
        
        // If the vision system has a valid pose, use it to correct the dead-wheel pose.
        if (limelightPoseData.isPresent()) {
            LimelightAprilTagLocalizer.LimelightPoseData data = limelightPoseData.get();
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

        // If we haven't gotten a vision update yet, but the dead wheels are giving us
        // valid numbers, then we can consider the pose reliable for driving.
        if (!isPoseReliable && !pinpoint.isNAN()) {
            isPoseReliable = true;
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
