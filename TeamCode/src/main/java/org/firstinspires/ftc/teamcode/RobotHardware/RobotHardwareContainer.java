package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomPinpointLocalizer;

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
    public final MecanumHardware mecanumDrive;
    public DriverAssist driverAssist;

    // Localizers for Pedro Pathing
    public final CustomPinpointLocalizer pinpoint;
    public final Localizer localizer; // The single, authoritative localizer

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create instances of each hardware class
        intake = new IntakeHardware();
        transfer = new TransferHardware();
        scoop = new ScoopHardware();
        diverter = new DiverterHardware();
        mecanumDrive = new MecanumHardware();

        // For testing, we are ONLY using the dead-wheel localizer.
        pinpoint = new CustomPinpointLocalizer(hardwareMap, new CustomPinpointLocalizer.CustomPinpointConstants());
        localizer = pinpoint; // Use pinpoint directly

        // Initialize all mechanical subsystems
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        scoop.init(hardwareMap);
        diverter.init(hardwareMap);
        mecanumDrive.init(hardwareMap);
    }

    public void initTurret(Follower follower, HardwareMap hardwareMap) {
        this.turret = new TurretHardware(follower);
        this.turret.init(hardwareMap);
    }

    public void initLauncher(Follower follower, TurretHardware turret, HardwareMap hardwareMap) {
        this.launcher = new LauncherHardware(follower, turret);
        this.launcher.init(hardwareMap);
    }

    public void initDriverAssist(Follower follower) {
        this.driverAssist = new DriverAssist(follower);
    }
}
