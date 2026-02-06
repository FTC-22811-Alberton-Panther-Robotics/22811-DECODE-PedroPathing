package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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
 * ---------------------------------------------------------------------------------
 * This class now accepts a pre-initialized Limelight3A object to prevent hardware conflicts.
 */
public class LimelightBallDetector {

    private Limelight3A limelight;

    // --- State Machine for Non-Blocking Detection ---
    private enum DetectionState { IDLE, WAITING_FOR_GREEN, WAITING_FOR_PURPLE }
    private DetectionState state = DetectionState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private static final double PIPELINE_SWITCH_DELAY_MS = 250.0;

    private Optional<LLResultTypes.DetectorResult> lastGreenArtifact = Optional.empty();
    private Optional<LLResultTypes.DetectorResult> lastPurpleArtifact = Optional.empty();

    public enum DetectedBall { GREEN, PURPLE, NONE }
    private DetectedBall detectedBall = DetectedBall.NONE;

    // init() now accepts a Limelight3A object instead of creating a new one.
    public void init(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        if (this.limelight != null) {
            if (telemetry != null) telemetry.addLine("Limelight Artifact Detector Initialized Successfully");
        } else {
            if (telemetry != null) telemetry.addLine("!!! LIMELIGHT NOT FOUND - CHECK CONFIGURATION !!!");
        }
    }

    /**
     * This update method must be called in the main loop to drive the state machine.
     */
    public void update() {
        if (limelight == null) return;

        switch (state) {
            case IDLE:
                setPipeline(1); // Switch to Green pipeline
                timer.reset();
                state = DetectionState.WAITING_FOR_GREEN;
                break;

            case WAITING_FOR_GREEN:
                if (timer.milliseconds() > PIPELINE_SWITCH_DELAY_MS) {
                    lastGreenArtifact = getLargestArtifactInternal();
                    setPipeline(2); // Switch to Purple pipeline
                    timer.reset();
                    state = DetectionState.WAITING_FOR_PURPLE;
                }
                break;

            case WAITING_FOR_PURPLE:
                if (timer.milliseconds() > PIPELINE_SWITCH_DELAY_MS) {
                    lastPurpleArtifact = getLargestArtifactInternal();
                    compareResults(); // Process the results
                    state = DetectionState.IDLE; // Reset for the next cycle
                }
                break;
        }
    }

    private void compareResults() {
        if (lastGreenArtifact.isPresent() && lastPurpleArtifact.isPresent()) {
            if (lastGreenArtifact.get().getTargetArea() > lastPurpleArtifact.get().getTargetArea()) {
                detectedBall = DetectedBall.GREEN;
            } else {
                detectedBall = DetectedBall.PURPLE;
            }
        } else if (lastGreenArtifact.isPresent()) {
            detectedBall = DetectedBall.GREEN;
        } else if (lastPurpleArtifact.isPresent()) {
            detectedBall = DetectedBall.PURPLE;
        } else {
            detectedBall = DetectedBall.NONE;
        }
    }

    private void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    private Optional<LLResultTypes.DetectorResult> getLargestArtifactInternal() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            return result.getDetectorResults().stream()
                    .max(Comparator.comparing(LLResultTypes.DetectorResult::getTargetArea));
        }
        return Optional.empty();
    }

    /**
     * Returns the last fully determined color from the detection state machine.
     * This method is non-blocking.
     */
    public DetectedBall getDetectedColor() {
        return detectedBall;
    }
}
