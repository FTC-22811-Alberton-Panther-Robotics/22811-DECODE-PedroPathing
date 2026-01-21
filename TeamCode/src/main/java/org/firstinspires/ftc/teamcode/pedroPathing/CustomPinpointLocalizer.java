package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.ArrayDeque;
import java.util.OptionalDouble;

/**
 * This is our custom implementation of the dead-wheel localizer, which wraps the
 * GoBildaPinpointDriver. It is responsible for two key things:
 * 1. Providing a clean interface between the Pedro Pathing library and the specific
 *    odometry hardware on the robot.
 * 2. Maintaining a short-term history of the robot's pose. This is essential for
 *    the `CombinedLocalizer` to be able to correct for vision system latency.
 * <p>
 * ---------------------------------------------------------------------------------
 * --- HOW IT WORKS ---
 * ---------------------------------------------------------------------------------
 * The `update()` method is called in every loop. Each time, it adds the robot's current
 * pose and a timestamp to a short list (`poseHistory`).
 * <p>
 * When the `CombinedLocalizer` receives a pose from the Limelight, it can ask this class,
 * "Where were you at time X in the past?" using the `getPoseAtLatency()` method.
 * This class then looks through its history to find the closest pose it has to that time.
 * This ability to look back in time is the key to accurately fusing the fast-but-drifty
 * dead-wheels with the slow-but-accurate camera.
 * ---------------------------------------------------------------------------------
 */
public class CustomPinpointLocalizer implements Localizer {

    private final GoBildaPinpointDriver pinpointDriver;
    private final CustomPinpointConstants constants;
    private final DistanceUnit distanceUnit;
    private final AngleUnit angleUnit;
    private final UnnormalizedAngleUnit unnormalizedAngleUnit;

    private final ArrayDeque<PoseSnapshot> poseHistory = new ArrayDeque<>();

    public static class PoseSnapshot {
        public final long timestamp;
        public final Pose pose;
        public PoseSnapshot(long timestamp, Pose pose) {
            this.timestamp = timestamp;
            this.pose = pose;
        }
    }

    public CustomPinpointLocalizer(HardwareMap hardwareMap, CustomPinpointConstants constants) {
        this.constants = constants;
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, constants.hardwareMapName);

        this.distanceUnit = constants.distanceUnit;
        this.angleUnit = AngleUnit.RADIANS;
        this.unnormalizedAngleUnit = this.angleUnit.getUnnormalized();

        pinpointDriver.setOffsets(constants.strafePodX, constants.forwardPodY, this.distanceUnit);
        pinpointDriver.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);
        pinpointDriver.setEncoderResolution(constants.encoderResolution);
        if (constants.yawScalar.isPresent()) {
            pinpointDriver.setYawScalar(constants.yawScalar.getAsDouble());
        }
    }

    @Override
    public void update() {
        pinpointDriver.update();
        poseHistory.addLast(new PoseSnapshot(System.nanoTime(), getPose()));
        if (poseHistory.size() > 150) { // Keep a buffer of around 150 readings
            poseHistory.removeFirst();
        }
    }

    @Override
    public Pose getPose() {
        return new Pose(
                pinpointDriver.getPosX(distanceUnit),
                pinpointDriver.getPosY(distanceUnit),
                pinpointDriver.getHeading(angleUnit)
        );
    }

    @Override
    public void setPose(Pose setPose) {
        Pose2D ftcPose = new Pose2D(distanceUnit, setPose.getX(), setPose.getY(), angleUnit, setPose.getHeading());
        pinpointDriver.setPosition(ftcPose);
        clearPoseHistory();
    }

    @Override
    public Pose getVelocity() {
        return new Pose(
                pinpointDriver.getVelX(distanceUnit),
                pinpointDriver.getVelY(distanceUnit),
                pinpointDriver.getHeadingVelocity(unnormalizedAngleUnit)
        );
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public double getTotalHeading() {
        return pinpointDriver.getHeading(unnormalizedAngleUnit);
    }

    @Override
    public void resetIMU() throws InterruptedException {
        pinpointDriver.recalibrateIMU();
        Thread.sleep(250);
    }

    @Override
    public double getIMUHeading() {
        return pinpointDriver.getHeading(angleUnit);
    }

    @Override
    public boolean isNAN() {
        return pinpointDriver.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ ||
               Double.isNaN(pinpointDriver.getPosX(distanceUnit)) ||
               Double.isNaN(pinpointDriver.getPosY(distanceUnit)) ||
               Double.isNaN(pinpointDriver.getHeading(angleUnit));
    }

    @Override
    public double getForwardMultiplier() { return 1.0; }

    @Override
    public double getLateralMultiplier() { return 1.0; }

    @Override
    public double getTurningMultiplier() { return 1.0; }

    /**
     * Gets the historical pose from the stored buffer that is closest to a given latency.
     * @param latency The age of the desired pose in seconds.
     * @return The historical Pose, or the current pose if no suitable history is found.
     */
    public Pose getPoseAtLatency(double latency) {
        long timeOfMeasurement = System.nanoTime() - (long)(latency * 1e9);
        PoseSnapshot closestSnapshot = null;
        for (PoseSnapshot snapshot : poseHistory) {
            if (snapshot.timestamp <= timeOfMeasurement) {
                closestSnapshot = snapshot;
            } else {
                break;
            }
        }
        return (closestSnapshot != null) ? closestSnapshot.pose : getPose();
    }

    /**
     * Clears the pose history buffer. This is called by the CombinedLocalizer after a
     * vision correction has been applied to prevent re-applying the same correction.
     */
    public void clearPoseHistory() {
        poseHistory.clear();
    }
}
