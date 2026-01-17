package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
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
    public LauncherHardware launcher;
    public final TransferHardware transfer;
    public final ScoopHardware scoop;
    public final DiverterHardware diverter;
    public TurretHardware turret;
    public final MecanumHardware mecanumDrive; // Added MecanumHardware

    // Localizers for Pedro Pathing
    public final LimelightAprilTagLocalizer aprilTag;
    public final CustomPinpointLocalizer pinpoint;
    public final Localizer localizer; // The single, authoritative localizer

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create instances of each hardware class
        intake = new IntakeHardware();
        transfer = new TransferHardware();
        scoop = new ScoopHardware();
        diverter = new DiverterHardware();
        mecanumDrive = new MecanumHardware(); // Instantiate MecanumHardware

        // Create the two underlying localizers.
        aprilTag = new LimelightAprilTagLocalizer();
        aprilTag.init(hardwareMap, telemetry);
        pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointLocalizer.CustomPinpointConstants());

        // Create the CombinedLocalizer
        localizer = new CombinedLocalizer(pinpoint, aprilTag, telemetry);

        // Initialize all mechanical subsystems
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        scoop.init(hardwareMap);
        diverter.init(hardwareMap);
        mecanumDrive.init(hardwareMap); // Initialize MecanumHardware
    }

    public void initTurret(Follower follower, HardwareMap hardwareMap) {
        this.turret = new TurretHardware(follower);
        this.turret.init(hardwareMap);
    }

    public void initLauncher(Follower follower, TurretHardware turret, HardwareMap hardwareMap) {
        this.launcher = new LauncherHardware(follower, turret);
        this.launcher.init(hardwareMap);
    }
}
