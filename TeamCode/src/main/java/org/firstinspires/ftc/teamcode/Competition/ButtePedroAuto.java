package org.firstinspires.ftc.teamcode.Competition;

// These are the required imports for a PedroPathing autonomous routine.
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is a configurable autonomous OpMode that uses the PedroPathing library.
 * It is structured as a Finite State Machine (FSM) to handle both the robot's
 * movement (pathing) and its actions (like running an intake or outtake).
 *
 * During the init phase, use the gamepad to configure the autonomous settings.
 * This example is heavily commented to guide students on how to create their own routines.
 */
@Autonomous(name = "CONFIGURABLE Pedro Auto", group = "Pedro", preselectTeleOp = "Butte2Controller")
public class ButtePedroAuto extends OpMode {

    RobotHardwareContainer robot;

    // ========== CONFIGURABLE SETTINGS ==========

    // Enums are a clean way to represent a fixed set of options.
    private enum Alliance {
        RED, BLUE
    }

    private enum StartPosition {
        FRONT, BACK
    }

    private enum AutoPath {
        SCORE_AND_PARK, CYCLE
    }

    // These variables will hold our selected options.
    private Alliance alliance = Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    private AutoPath autoPath = AutoPath.SCORE_AND_PARK;

    // Pose list
    private static Pose BLUE_FRONT_START = new Pose(56, 9, Math.toRadians(90));
    private Pose BLUE_BACK_START = new Pose(33, 135, Math.toRadians(270));
    private Pose RED_FRONT_START = new Pose(88, 9, Math.toRadians(90));
    private Pose RED_BACK_START = new Pose(111, 135, Math.toRadians(270));

    private Pose BLUE_SCORE_POSE = new Pose(60, 84, Math.toRadians(135));
    private Pose BLUE_PICKUP_FRONT_SPIKE = new Pose(40, 36, Math.toRadians(0));
    private Pose BLUE_PICKUP_MIDDLE_SPIKE = new Pose(40, 60, Math.toRadians(0));
    private Pose BLUE_PICKUP_BACK_SPIKE = new Pose(40, 84, Math.toRadians(0));

    private Pose RED_SCORE_POSE = new Pose(84, 84, Math.toRadians(45));
    private Pose RED_PICKUP_FRONT_SPIKE = new Pose(40, 36, Math.toRadians(0));
    private Pose RED_PICKUP_MIDDLE_SPIKE = new Pose(40, 60, Math.toRadians(0));
    private Pose RED_PICKUP_BACK_SPIKE = new Pose(40, 84, Math.toRadians(0));
    private Pose BLUE_AUTO_PARK = new Pose(60, 60, Math.toRadians(180));
    private Pose RED_AUTO_PARK = new Pose(84, 60, Math.toRadians(0));




    // Gamepad state tracking to prevent multiple button presses on one click.
    private Gamepad.RumbleEffect customRumbleEffect;
    private boolean dpad_up_pressed = false;
    private boolean dpad_down_pressed = false;
    private boolean dpad_left_pressed = false;
    private boolean dpad_right_pressed = false;
    private boolean left_bumper_pressed = false;
    private boolean right_bumper_pressed = false;


    // ========== DECLARE OPMODE MEMBERS ==========

    private Follower follower;
    private ElapsedTime pathTimer;
    private ElapsedTime actionTimer;
    private int pathState;
    private int actionState;

    // === PATHING ===
    // We will now calculate the startPose in init() based on the selected options.
    private Pose startPose;
    private Pose pickupFrontPose;
    private Pose pickupMiddlePose;
    private Pose pickupBackPose;
    private Pose scorePose;
    private Pose parkPose;

    private Path scorePath;
    private Path parkPath;
    private PathChain cyclePath;


    // ========== OpMode METHODS ==========

    @Override
    public void init() {
        // Create a custom rumble effect for feedback during selection.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 200)  // Rumble both motors at 50% power for 200 ms
                .build();

        // Initialize our RobotHardwareContainer.
        robot = new RobotHardwareContainer(hardwareMap, telemetry);

        // Initialize timers and follower.
        pathTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("Autonomous Configuration:");
        telemetry.addLine("--------------------------------");
        telemetry.addLine("Press D-Pad Up/Down to change Alliance.");
        telemetry.addLine("Press Left/Right Bumper to change Start Position.");
        telemetry.addLine("Press A/B to change Auto Path."); // You can add this if needed
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // --- Gamepad Logic for Configuration ---

        // Alliance Selection (D-Pad Up/Down)
        if (gamepad1.dpad_left && !dpad_left_pressed) {
            alliance = Alliance.BLUE;
            gamepad1.runRumbleEffect(customRumbleEffect);
        } else if (gamepad1.dpad_right && !dpad_right_pressed) {
            alliance = Alliance.RED;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }

        // Start Position Selection (Bumpers)
        if (gamepad1.dpad_up && !dpad_up_pressed) {
            startPosition = StartPosition.FRONT;
            gamepad1.runRumbleEffect(customRumbleEffect);
        } else if (gamepad1.dpad_down && !dpad_down_pressed) {
            startPosition = StartPosition.BACK;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }

        // --- Update Button Pressed States ---
        dpad_up_pressed = gamepad1.dpad_up;
        dpad_down_pressed = gamepad1.dpad_down;
        dpad_left_pressed = gamepad1.dpad_left;
        dpad_right_pressed = gamepad1.dpad_right;

        // --- Telemetry for Configuration ---
        telemetry.clearAll();
        telemetry.addLine("--- Autonomous Configuration ---");
        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Start Position", startPosition.toString());
        telemetry.addData("Auto Path", autoPath.toString());
        telemetry.addLine("--------------------------------");
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        // --- Finalize Configuration and Build Paths ---
        // Now that settings are locked in, calculate poses and build paths.
        calculatePoses();
        buildPaths();

        // Set the robot's starting position.
        follower.setStartingPose(startPose);

        // Reset timers and set initial states.
        pathTimer.reset();
        actionTimer.reset();
        pathState = 0; // Will be set to 1 in loop
        actionState = 0;
        setPathState(1); // Kick off the state machine
    }

    /**
     * This method calculates the poses based on the alliance and start position.
     * The field is symmetrical, so we can flip coordinates for the red alliance.
     */
    private void calculatePoses() {
        double heading_offset = (alliance == Alliance.BLUE) ? 0 : -Math.PI;

        // Example poses - TUNE THESE FOR YOUR ROBOT AND GAME

        if (alliance == Alliance.BLUE && startPosition == StartPosition.FRONT) {
            startPose = BLUE_FRONT_START;
        } else if ( alliance == Alliance.BLUE && startPosition == StartPosition.BACK) {
            startPose = BLUE_BACK_START;
        } else if (alliance == Alliance.RED && startPosition == StartPosition.FRONT) {
            startPose = RED_FRONT_START;
        } else if (alliance == Alliance.RED && startPosition == StartPosition.BACK){
            startPose = RED_BACK_START;
        }

        scorePose = (alliance == Alliance.BLUE)? BLUE_SCORE_POSE: RED_SCORE_POSE;
        pickupFrontPose = (alliance == Alliance.BLUE)? BLUE_PICKUP_FRONT_SPIKE: RED_PICKUP_FRONT_SPIKE;
        pickupMiddlePose = (alliance == Alliance.BLUE)? BLUE_PICKUP_MIDDLE_SPIKE: RED_PICKUP_MIDDLE_SPIKE;
        pickupBackPose = (alliance == Alliance.BLUE)? BLUE_PICKUP_BACK_SPIKE: RED_PICKUP_BACK_SPIKE;
        parkPose = (alliance == Alliance.BLUE)? BLUE_AUTO_PARK: RED_AUTO_PARK;
    }


    // The rest of the methods (loop, stop, buildPaths, updatePath, etc.) follow below...
    // Note: I've updated buildPaths and updatePath to handle the different auto routines.

    @Override
    public void loop() {
        follower.update();
        updatePath();
        updateAction();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading (Degrees)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }


    // ========== HELPER METHODS ==========

    private void buildPaths() {
        // --- Score and Park Path ---
        scorePath = new Path(new BezierLine(startPose, scorePose));
        scorePath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        parkPath = new Path(new BezierLine(scorePose, parkPose));
        parkPath.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        // --- Cycle Path ---
        if (autoPath == AutoPath.CYCLE) {
            cyclePath = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickupFrontPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickupFrontPose.getHeading())
                    .build();
        }
    }

    private void updatePath() {
        switch (pathState) {
            case 0: // IDLE
                break;

            case 1: // Start first path (to score)
                follower.followPath(scorePath);
                setPathState(2);
                break;

            case 2: // Wait for score path to finish, then trigger score action
                if (!follower.isBusy()) {
                    setActionState(2); // Trigger scoring action
                    setPathState(3);
                }
                break;

            case 3: // Wait for score action to finish
                if (actionState == 0) {
                    // Decide what to do next based on the selected auto path
                    if (autoPath == AutoPath.SCORE_AND_PARK) {
                        follower.followPath(parkPath);
                        setPathState(10); // Go to parking state
                    } else { // CYCLE
                        follower.followPath(cyclePath);
                        setPathState(4); // Go to cycle states
                    }
                }
                break;

            // --- Cycle-Specific States ---
            case 4: // Wait for path to pickup to finish
                if (!follower.isBusy()) {
                    setActionState(1); // Trigger pickup action
                    setPathState(5);
                }
                break;
            case 5: // Wait for pickup action to finish
                if (actionState == 0) {
                    // Here you would build a path back to the score pose and follow it
                    // For simplicity, we'll end here.
                    setPathState(-1);
                }
                break;

            // --- Park-Specific State ---
            case 10: // Wait for park path to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // End of routine
                }
                break;

            case -1: // STOP
                follower.breakFollowing();
                break;
        }
    }

    private void updateAction() {
        switch (actionState) {
            case 0: // IDLE
                break;
            case 1: // Start pickup sequence
                robot.intake.run();
                actionTimer.reset();
                setActionState(10);
                break;
            case 10: // Wait for pickup to complete
                if (actionTimer.seconds() > 1.5) {
                    robot.intake.stop();
                    setActionState(0);
                }
                break;
            case 2: // Start scoring sequence
                robot.launcher.spinUp();
                actionTimer.reset();
                setActionState(20);
                break;
            case 20: // Wait for shooter to get to speed
                if (actionTimer.seconds() > 1.0) {
                    robot.transfer.run();
                    actionTimer.reset();
                    setActionState(21);
                }
                break;
            case 21: // Wait for artifact to be shot
                if (actionTimer.seconds() > 3) {
                    robot.launcher.stop();
                    robot.transfer.stop();
                    setActionState(0);
                }
                break;
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    private void setActionState(int aState) {
        actionState = aState;
        actionTimer.reset();
    }
}
