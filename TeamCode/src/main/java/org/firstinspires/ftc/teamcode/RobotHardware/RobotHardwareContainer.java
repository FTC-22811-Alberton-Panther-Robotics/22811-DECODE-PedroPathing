package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.CombinedLocalizer;

/**
 * This class acts as a central container to hold and initialize all the robot's hardware subsystems.
 * It follows the "Dependency Injection" pattern, meaning it creates and holds instances of
 * every hardware class (like IntakeHardware, LauncherHardware, etc.) in one place.
 * <p>
 * By creating all hardware objects here, any OpMode (like BozemanTeleop) can get access
 * to all the robot's hardware by creating just ONE instance of this container class.
 * This keeps the OpMode code cleaner and separates the hardware initialization logic
 * from the control logic.
 */
public class RobotHardwareContainer {

    // Publicly accessible hardware subsystem objects
    public final IntakeHardware intake;
    public LauncherHardware launcher;
    public final TransferHardware transfer;
    public final ScoopHardware scoop;
    public final DiverterHardware diverter;
    public TurretHardware turret;
    public final LimelightBallDetector limelightBallDetector;

    // The single, authoritative localizer for the entire robot.
    public final Localizer localizer;

    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // --- IMPORTANT: HARDWARE INITIALIZATION ORDER ---
        // To prevent hardware conflicts, any class that needs direct access to a device (like the Limelight)
        // must be created here and passed to any other classes that need it. Here, we create the CombinedLocalizer
        // first, which handles all AprilTag and odometry. We then pass its Limelight instance to the
        // ball detector to avoid a resource conflict.

        // 1. Create the authoritative localizer. This claims the Limelight for AprilTags.
        localizer = new CombinedLocalizer(hardwareMap, telemetry);

        // 2. Create the ball detector and give it the Limelight instance that the localizer is already using.
        limelightBallDetector = new LimelightBallDetector();
        // --- HARDWARE FIX: Correctly access the Limelight from the concrete class ---
        // We must cast the `localizer` (which is of the `Localizer` interface type) to its actual
        // class, `CombinedLocalizer`, to access the `getLimelight()` method.
        limelightBallDetector.init(((CombinedLocalizer) localizer).getLimelight(), telemetry);

        // 3. Create all other hardware instances.
        intake = new IntakeHardware();
        transfer = new TransferHardware();
        scoop = new ScoopHardware();
        diverter = new DiverterHardware();

        // 4. Initialize all mechanical subsystems.
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        scoop.init(hardwareMap);
        diverter.init(hardwareMap, limelightBallDetector);
    }

    public void initTurret(Follower follower, HardwareMap hardwareMap) {
        this.turret = new TurretHardware(follower);
        this.turret.init(hardwareMap);
    }

    public void initLauncher(Follower follower, HardwareMap hardwareMap) {
        this.launcher = new LauncherHardware(follower);
        this.launcher.init(hardwareMap);
    }
}
