package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomPinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.LimelightAprilTagLocalizer;

/**
 * This class acts as a container to hold and initialize all the robot's hardware subsystems.
 */
public class RobotHardwareContainer {

    // Publicly accessible hardware subsystem objects
    public final IntakeHardware intake;
    public final LauncherHardware launcher;
    public final BallDiverterHardware ballDiverter;
    public final BallTransferHardware ballTransfer;
    public final ScooperHardware scooper;
    public final ColorDiverterHardware colorDiverter; // No longer nullable

    // Localizers for Pedro Pathing
    public final LimelightAprilTagLocalizer aprilTag;
    public final CustomPinpointLocalizer pinpoint;
    public final Localizer localizer; // The single, authoritative localizer

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create instances of each hardware class
        intake = new IntakeHardware();
        launcher = new LauncherHardware();
        ballDiverter = new BallDiverterHardware();
        ballTransfer = new BallTransferHardware();
        scooper = new ScooperHardware();
        colorDiverter = new ColorDiverterHardware();

        // Create the two underlying localizers
        aprilTag = new LimelightAprilTagLocalizer();
        aprilTag.init(hardwareMap, telemetry);
        // CORRECTED: Initialize the pinpoint localizer with its own nested constants class.
        pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointLocalizer.CustomPinpointConstants());

        // Create the CombinedLocalizer, which fuses the two data sources
        localizer = new CombinedLocalizer(pinpoint, aprilTag, telemetry);

        // Call the init() method for each mechanical subsystem
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        ballDiverter.init(hardwareMap);
        ballTransfer.init(hardwareMap);
        scooper.init(hardwareMap);
        colorDiverter.init(hardwareMap);
    }
}
