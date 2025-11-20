package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Pedro Pathing imports
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.FTCCoordinates;

import java.util.List;
import java.util.Optional;

/**
 * Manages all interactions with the Limelight 3A camera for the DECODE game season.
 * This class handles initialization, AprilTag detection, and provides methods
 * to get detected tag IDs and the robot's field-relative pose.
 */
public class LimelightAprilTagLocalizer {

    private Limelight3A limelight;
    private Telemetry telemetry;

    public static final int MOTIF_GPP_ID = 21; // GPP Motif
    public static final int MOTIF_PGP_ID = 22; // PGP Motif
    public static final int MOTIF_PPG_ID = 23; // PPG Motif

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            telemetry.addLine("Limelight Initialized Successfully");
        } catch (Exception e) {
            limelight = null;
            telemetry.addLine("!!! LIMELIGHT NOT FOUND - CHECK CONFIGURATION !!!");
        }
    }

    public void start() {
        if (limelight != null) {
            limelight.start();
        }
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Gets the ID of the first valid AprilTag the Limelight detects.
     * @return An Optional containing the ID, or an empty Optional if no tag is found.
     */
    public Optional<Integer> getDetectedTagId() {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                LLResultTypes.FiducialResult firstTag = fiducialResults.get(0);
                int detectedId = firstTag.getFiducialId();

                telemetry.addData("Limelight Detected ID", detectedId);
                return Optional.of(detectedId);
            }
        }

        telemetry.addData("Limelight Detected ID", "None");
        return Optional.empty();
    }

    /**
     * Gets the robot's field-relative pose based on the primary AprilTag detection.
     * @return An Optional containing the robot's calculated Pose, or an empty Optional if no valid pose can be determined.
     */
    public Optional<Pose> getRobotPose() {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();

            // The botpose will not be null if a pose is available, but its fields might be zero.
            if (botpose != null) {

                // DEFINITIVELY CORRECTED: Use getPosition() and getOrientation() as defined in the Pose3D class.
                Position position = botpose.getPosition();
                YawPitchRollAngles orientation = botpose.getOrientation();

                // Check if the pose is non-zero to ensure it's a valid calculation
                if (position.x != 0.0 || position.y != 0.0) {
                    // The Limelight SDK provides the pose in the standard FTC coordinate system.
                    // We must create a PedroPathing Pose object and specify its coordinate system.
                    Pose ftcPose = new Pose(
                            position.x, // Get X from the Position object
                            position.y, // Get Y from the Position object
                            orientation.getYaw(AngleUnit.RADIANS), // Get heading (Yaw) from the YawPitchRollAngles object
                            FTCCoordinates.INSTANCE // Specify that these numbers are in the FTC coordinate system
                    );

                    // Now, use the built-in PedroPathing converter to get the pose in Pedro's system.
                    Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

                    telemetry.addData("Limelight Pose (X, Y, H)", "%.2f, %.2f, %.1f",
                            pedroPose.getX(), pedroPose.getY(), Math.toDegrees(pedroPose.getHeading()));

                    return Optional.of(pedroPose);
                }
            }
        }

        telemetry.addData("Limelight Pose", "None");
        return Optional.empty();
    }
}
