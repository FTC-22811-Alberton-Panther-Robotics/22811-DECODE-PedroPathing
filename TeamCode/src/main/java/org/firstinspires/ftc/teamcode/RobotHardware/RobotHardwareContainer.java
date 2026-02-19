package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {

        // 3. Create all other hardware instances.
        intake = new IntakeHardware();
        transfer = new TransferHardware();
        scoop = new ScoopHardware();
        diverter = new DiverterHardware();

        // 4. Initialize all mechanical subsystems.
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        scoop.init(hardwareMap);
        diverter.init(hardwareMap);
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
