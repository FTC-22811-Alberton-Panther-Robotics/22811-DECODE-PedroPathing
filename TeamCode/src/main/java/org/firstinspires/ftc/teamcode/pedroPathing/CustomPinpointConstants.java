package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * This class holds all the physical constants for the Pinpoint dead-wheel localizer.
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * These constants define the physical layout of your dead-wheel odometry pods.
 * They MUST be measured accurately for localization to work.
 *
 * 1. Pod Positions: `forwardPodY` and `strafePodX` are the distances of your odometry
 *    pods from the robot's center of rotation. They must be measured carefully with a
 *    ruler or calipers.
 *      - `forwardPodY`: The perpendicular distance from the center of rotation to the
 *        center of the forward-facing odometry pod.
 *      - `strafePodX`: The perpendicular distance from the center of rotation to the
 *        center of the sideways-facing odometry pod.
 *
 * 2. Encoder Directions: If your odometry pods are mounted backwards or upside down,
 *    you may need to reverse their counting direction. If the robot's position drifts
 *    in the wrong direction when you move it, you need to reverse the corresponding
 *    encoder direction.
 *
 * 3. Yaw Scalar: This is an advanced tuning value for three-wheel odometry setups.
 *    Since this robot uses two-wheel odometry with the drive train IMU for heading,
 *    this value is not used and is left empty.
 * ---------------------------------------------------------------------------------
 */
public class CustomPinpointConstants {

    // TODO: Verify these measurements on the physical robot.
    public double forwardPodY = -8.5;
    public double strafePodX = 2.5;
    
    public DistanceUnit distanceUnit = DistanceUnit.INCH;
    public String hardwareMapName = "pinpoint";
    public OptionalDouble yawScalar = OptionalDouble.empty(); // Not used for 2-wheel odom
    
    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    
    // TODO: Test and reverse these if odometry drifts in the wrong direction.
    public GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public CustomPinpointConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public CustomPinpointConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public CustomPinpointConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public CustomPinpointConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public CustomPinpointConstants yawScalar(double yawScalar) {
        this.yawScalar = OptionalDouble.of(yawScalar);
        return this;
    }

    public CustomPinpointConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public CustomPinpointConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public CustomPinpointConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }
}
