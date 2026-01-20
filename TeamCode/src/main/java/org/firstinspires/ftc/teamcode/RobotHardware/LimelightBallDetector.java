package org.firstinspires.ftc.teamcode.RobotHardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class LimelightBallDetector {

    private Limelight3A limelight;
    private Telemetry telemetry;

    // Define the pipeline indices for different ball detection modes
    private static final int GREEN_BALL_PIPELINE = 1;
    private static final int PURPLE_BALL_PIPELINE = 2;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            if (telemetry != null) {
                telemetry.addLine("Limelight Ball Detector Initialized Successfully");
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

    public Optional<LLResult.Target> getLargestBall() {
        if (limelight == null) {
            return Optional.empty();
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getTargets().isEmpty()) {
            // The targets are already sorted by area by default, so the first one is the largest
            return Optional.of(result.getTargets().get(0));
        }

        return Optional.empty();
    }
}
