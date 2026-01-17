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
 * A standalone dead-wheel localizer that correctly wraps the GoBildaPinpointDriver.
 * This class now contains its own constants for clean encapsulation.
 */
public class CustomPinpointLocalizer implements Localizer {

    /**
     * Nested static class for all configuration specific to the Pinpoint hardware.
     */
    public static class CustomPinpointConstants {
        public String hardwareMapName = "pinpoint";
        public double forwardPodY = -8.5; // Y offset from center of rotation
        public double strafePodX = 2.5;   // X offset from center of rotation
        public double encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public DistanceUnit distanceUnit = DistanceUnit.INCH;
        public OptionalDouble yawScalar = OptionalDouble.empty(); // Only use if significant IMU drift is observed
    }

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
        if (poseHistory.size() > 150) {
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

    public void clearPoseHistory() {
        poseHistory.clear();
    }
}
