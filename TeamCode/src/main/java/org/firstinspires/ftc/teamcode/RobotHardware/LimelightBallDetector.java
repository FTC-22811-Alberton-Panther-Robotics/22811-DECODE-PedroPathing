package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Comparator;
import java.util.Optional;

/**
 * Manages all Limelight-based detection of game pieces (Artifacts).
 * This class is responsible for switching pipelines and parsing results for consumption
 * by other hardware classes.
 *
 * ---------------------------------------------------------------------------------
 * --- TESTING, TUNING, AND CONFIGURATION ---
 * ---------------------------------------------------------------------------------
 * 1. Limelight Web Interface: Access at http://limelight.local:5801
 * 2. Pipeline Setup:
 *    - Pipeline 1: Must be a "Detector" pipeline tuned to find GREEN Artifacts.
 *    - Pipeline 2: Must be a "Detector" pipeline tuned to find PURPLE Artifacts.
 * 3. ROI (Region of Interest):
 *    - The logic in this class is designed to find artifacts within a specific vertical
 *      slice of the screen. This is critical for ignoring artifacts already inside the robot.
 *    - The coordinates for this ROI are set in the `DiverterHardware` class.
 * ---------------------------------------------------------------------------------
 */
public class LimelightBallDetector {

    private Limelight3A limelight;
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (telemetry != null) {
                telemetry.addLine("Limelight Artifact Detector Initialized Successfully");
            }
        } catch (Exception e) {
            limelight = null;
            if (telemetry != null) {
                telemetry.addLine("!!! LIMELIGHT NOT FOUND - CHECK CONFIGURATION !!!");
            }
        }
    }

    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Gets the largest detected Artifact within a specified vertical region of the screen (Region of Interest).
     * This is used to focus detection on new Artifacts outside the robot, ignoring any that might
     * already be on the intake ramp.
     *
     * @param minY The minimum Y-pixel coordinate for the ROI (can be negative, represents top of region).
     * @param maxY The maximum Y-pixel coordinate for the ROI (can be positive, represents bottom of region).
     * @return An Optional containing the DetectorResult with the largest area within the ROI, or an empty Optional.
     */
    public Optional<LLResultTypes.DetectorResult> getLargestArtifactInROI(double minY, double maxY) {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            // Filter the results to only include objects within the specified vertical ROI,
            // then find the one with the largest area among them.
            return result.getDetectorResults().stream()
                    .filter(target -> {
                        // The 'getTargetYPixels()' value is the vertical offset from the crosshair in pixels.
                        // It can be positive (down) or negative (up).
                        double y = target.getTargetYPixels();
                        return y >= minY && y <= maxY;
                    })
                    .max(Comparator.comparing(LLResultTypes.DetectorResult::getTargetArea));
        }

        return Optional.empty();
    }
}
