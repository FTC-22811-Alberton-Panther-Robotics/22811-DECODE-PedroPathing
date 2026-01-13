package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.BallDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.ColorDiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.intakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.HARDWARE.TransferHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.LimelightAprilTagLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomPinpointLocalizer;

/**
 * This class acts as a container to hold and initialize all the robot's hardware subsystems.
 */
public class RobotHardwareContainer {

    // Publicly accessible hardware subsystem objects

    public final BallDiverterHardware ballDiverter;
    public final intakeHardware intake;
    public final LauncherHardware launcher;
    public final TransferHardware transfer;
    public final LimelightAprilTagLocalizer aprilTag;
    public final CustomPinpointLocalizer pinpoint;

    // The ColorDiverter is nullable because it only exists on the competition bot
    public ColorDiverterHardware colorDiverter;

    // The single, authoritative localizer for the robot
    public final Localizer localizer;

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create instances of each hardware class
        ballDiverter = new BallDiverterHardware();
        intake = new intakeHardware();
        launcher = new LauncherHardware();
        transfer = new TransferHardware();
        
        // Create the two underlying localizers
        aprilTag = new LimelightAprilTagLocalizer();
        aprilTag.init(hardwareMap, telemetry);
        pinpoint = new CustomPinpointLocalizer(hardwareMap, Constants.localizerConstants);

        // Create the CombinedLocalizer, which fuses the two data sources
        localizer = new CombinedLocalizer(pinpoint, aprilTag, telemetry);

        // Call the init() method for each mechanical subsystem
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        transfer.init(hardwareMap);

        // --- Conditional Initialization for Competition Bot Hardware ---
        if (Constants.IS_COMPETITION_BOT) {
            colorDiverter = new ColorDiverterHardware();
            colorDiverter.init(hardwareMap);
        }
    }
}
