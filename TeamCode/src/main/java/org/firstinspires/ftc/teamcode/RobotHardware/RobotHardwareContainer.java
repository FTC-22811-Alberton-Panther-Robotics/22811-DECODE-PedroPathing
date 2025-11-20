package org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.LauncherHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.TransferHardware;

/**
 * This class acts as a container to hold and initialize all the robot's hardware subsystems.
 * By creating an instance of this class in an OpMode, you gain access to all robot
 * mechanisms without having to initialize each one individually.
 */
public class RobotHardwareContainer {

    // Publicly accessible hardware subsystem objects
    public final IntakeHardware intake;
    public final LauncherHardware launcher;
    public final TransferHardware transfer;
    public final LimelightAprilTagLocalizer aprilTag;

    /**
     * The constructor for the RobotHardwareContainer. It initializes all hardware subsystems.
     * @param hardwareMap The hardwareMap from the OpMode, used to map motor and servo names.
     */
    public RobotHardwareContainer(HardwareMap hardwareMap, Telemetry telemetry) {
        // Create new instances of each hardware class
        intake = new IntakeHardware();
        launcher = new LauncherHardware();
        transfer = new TransferHardware();
        aprilTag = new LimelightAprilTagLocalizer();

        // Call the init() method for each subsystem, passing the hardwareMap
        intake.init(hardwareMap);
        launcher.init(hardwareMap);
        transfer.init(hardwareMap);
        aprilTag.init(hardwareMap, telemetry);
    }
}
