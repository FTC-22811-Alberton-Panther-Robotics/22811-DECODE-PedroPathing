package org.firstinspires.ftc.teamcode.InDevelopment;

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

// By importing our new FieldPosePresets class, we can access all the poses directly.
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;

/**
 * This is a configurable autonomous OpMode that uses the PedroPathing library.
 * It is structured as a Finite State Machine (FSM) to handle both the robot's
 * movement (pathing) and its actions (like running an intake or outtake).
 *
 * During the init phase, use the gamepad to configure the autonomous settings.
 * This example is heavily commented to guide students on how to create their own routines.
 */
@Autonomous(name = "Configurable Pedro Auto", group = "Pedro", preselectTeleOp = "TeleopManualControls")
public class ConfigurablePedroAuto extends OpMode {

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

    // The pose declarations have been moved to the FieldPosePresets.java file!

    // Gamepad state tracking to prevent multiple button presses on one click.
    private Gamepad.RumbleEffect customRumbleEffect;
    private boolean dpad_up_pressed = false;
    private boolean dpad_down_pressed = false;
    private boolean dpad_left_pressed = false;
    private boolean dpad_right_pressed = false;

    // ========== DECLARE OPMODE MEMBERS ==========

    private Follower follower;
    private ElapsedTime pathTimer;
    private ElapsedTime actionTimer;
    private int pathState;
    private int actionState;

    // === PATHING ===
    // We will now calculate the poses in start() based on the selected options.
    private Pose startPose;
    private Pose pickupFrontPose;
    private Pose pickupMiddlePose;
    private Pose pickupBackPose;
    private Pose scorePose;
    private Pose parkPose;

    // scorePath is now a PathChain to allow for control points.
    private PathChain scorePath;
    private Path parkPath;
    private PathChain cyclePath;

    // ========== OpMode METHODS ==========

    @Override
    public void init() {
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 200)
                .build();

        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        pathTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        // Pass the fused localizer from the RobotHardwareContainer to the Follower
        follower = Constants.createFollower(hardwareMap, robot.localizer);

        telemetry.addLine("Autonomous Configuration:");
        telemetry.addLine("--------------------------------");
        telemetry.addLine("Press D-Pad Left/Right for Alliance.");
        telemetry.addLine("Press D-Pad Up/Down for Start Position.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left && !dpad_left_pressed) {
            alliance = Alliance.BLUE;
            gamepad1.runRumbleEffect(customRumbleEffect);
        } else if (gamepad1.dpad_right && !dpad_right_pressed) {
            alliance = Alliance.RED;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }

        if (gamepad1.dpad_up && !dpad_up_pressed) {
            startPosition = StartPosition.FRONT;
            gamepad1.runRumbleEffect(customRumbleEffect);
        } else if (gamepad1.dpad_down && !dpad_down_pressed) {
            startPosition = StartPosition.BACK;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }

        dpad_up_pressed = gamepad1.dpad_up;
        dpad_down_pressed = gamepad1.dpad_down;
        dpad_left_pressed = gamepad1.dpad_left;
        dpad_right_pressed = gamepad1.dpad_right;

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
        calculatePoses();
        buildPaths();
        follower.setStartingPose(startPose);

        pathTimer.reset();
        actionTimer.reset();
        pathState = 0;
        actionState = 0;
        setPathState(1);
    }

    private void calculatePoses() {
        if (alliance == Alliance.BLUE) {
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
            scorePose = FieldPosePresets.BLUE_SCORE_POSE;
            pickupFrontPose = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
            pickupMiddlePose = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
            pickupBackPose = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
        } else { // RED Alliance
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
            scorePose = FieldPosePresets.RED_SCORE_POSE;
            pickupFrontPose = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
            pickupMiddlePose = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
            pickupBackPose = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
            parkPose = FieldPosePresets.RED_AUTO_PARK;
        }
    }

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

    private void buildPaths() {
        // --- Score and Park Path ---
        // CORRECTED EXAMPLE of using a control point to shape the path.
        // To force the path to curve through a specific point, we create a PathChain
        // with two BezierLine segments. The first goes from the start to the control point,
        // and the second goes from the control point to the end.
        //
        // Let's create a control point to keep the robot in the tile column next to the truss.
        // We'll define it based on the scoring pose's X-coordinate, and halfway down the field.
        Pose scoreControlPoint = new Pose(scorePose.getX(), 48, startPose.getHeading());

        scorePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scoreControlPoint))
                .addPath(new BezierLine(scoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // The park path is a simple, single segment, so it can remain a regular Path.
        parkPath = new Path(new BezierLine(scorePose, parkPose));
        parkPath.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        // --- Cycle Path ---
        if (autoPath == AutoPath.CYCLE) {
            // We do the same thing for the cycle path to control its shape.
            Pose pickupControlPoint = new Pose(pickupFrontPose.getX(), 48, scorePose.getHeading());
            cyclePath = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickupControlPoint))
                    .addPath(new BezierLine(pickupControlPoint, pickupFrontPose))
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
                    if (autoPath == AutoPath.SCORE_AND_PARK) {
                        follower.followPath(parkPath);
                        setPathState(10);
                    } else { // CYCLE
                        follower.followPath(cyclePath);
                        setPathState(4);
                    }
                }
                break;
            case 4: // Wait for path to pickup to finish
                if (!follower.isBusy()) {
                    setActionState(1);
                    setPathState(5);
                }
                break;
            case 5: // Wait for pickup action to finish
                if (actionState == 0) {
                    setPathState(-1); // End of routine
                }
                break;
            case 10: // Wait for park path to finish
                if (!follower.isBusy()) {
                    setPathState(-1);
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
//                    robot.transfer.run();
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
