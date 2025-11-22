package org.firstinspires.ftc.teamcode.InDevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Import the hardware container and the action manager
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.MecanumHardware;

@TeleOp(name="ButteTeleop (Actions)", group="01 Butte")
@Disabled
public class TeleopWithActions extends OpMode {

    // --- Declare OpMode members ---
    RobotHardwareContainer robot;
    ActionManager actionManager;
    MecanumHardware mecanumHardware;

    // --- Private state variables for button presses ---
    private boolean dpad_right_pressed = false;
    private boolean y_pressed = false;

    @Override
    public void init(){
        // Initialize all hardware and the action manager
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        mecanumHardware = new MecanumHardware();
        mecanumHardware.init(hardwareMap);

        telemetry.addLine("Actions TeleOp Initialized. Ready to drive!");
        telemetry.update();
    }

    @Override
    public void loop(){
        // --- UPDATE MANAGERS ---
        // This is CRITICAL. The action manager's state machine must be updated every loop.
        actionManager.update();

        // --- DRIVER CONTROLS (Gamepad 1) ---

        // Drivetrain
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        mecanumHardware.drive(forward, strafe, turn);

        // --- Mechanism Controls ---
        if (gamepad1.right_trigger > 0.1) {
            // Right trigger runs the intake IN
            actionManager.startIntake();
        } else if (gamepad1.left_trigger > 0.1) {
            // CORRECTED: Left trigger now calls the unified reverseAll() method
            actionManager.reverseAll();
        } else if (gamepad1.dpad_right && !dpad_right_pressed) {
            // CORRECTED: 'startScoring' is now 'startLaunch'
            actionManager.startLaunch();
        } else if (gamepad1.y && !y_pressed) {
            // Transfer Control (This method no longer exists, let's re-evaluate)
            // For now, let's make 'Y' a manual transfer override
            robot.transfer.run();
        } else if (gamepad1.b) {
            // 'B' button is our emergency stop. It will halt all actions.
            actionManager.stopAll();
        } else {
            // If no buttons are pressed AND the manager is not busy with a timed sequence...
            if (!actionManager.isBusy()) {
                // ...stop all motors. This prevents the intake/transfer from running forever.
                actionManager.stopAll();
            }
        }

        // --- Update button state trackers ---
        dpad_right_pressed = gamepad1.dpad_right;
        y_pressed = gamepad1.y;


        // --- TELEMETRY ---
        telemetry.addData("Action Manager State", actionManager.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Left Flywheel RPM", "%.0f", robot.launcher.getLeftFlywheelRPM());
        telemetry.addData("Right Flywheel RPM", "%.0f", robot.launcher.getRightFlywheelRPM());
        telemetry.addData("Target RPM", robot.launcher.TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void stop(){
        // Ensure everything is stopped when the OpMode ends.
        actionManager.stopAll();
        mecanumHardware.drive(0,0,0);
    }
}
