//package org.firstinspires.ftc.teamcode.InDevelopment;
//
//// These are the required imports for a PedroPathing autonomous routine.
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.geometry.BezierLine;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
//import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
//import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//// By importing our new FieldPosePresets class, we can access all the poses directly.
//import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
//
//@Autonomous(name = "Configurable Pedro Auto", group = "Pedro", preselectTeleOp = "TeleopManualControls")
//public class ConfigurablePedroAuto extends OpMode {
//
//    RobotHardwareContainer robot;
//    ActionManager actionManager; // The new, unified action manager
//
//    // ========== CONFIGURABLE SETTINGS ==========
//    private enum StartPosition { FRONT, BACK }
//    private enum AutoPath { SCORE_AND_PARK, CYCLE }
//
//    // CORRECTED: Use the global GameState.Alliance for consistency
//    private GameState.Alliance alliance = GameState.Alliance.BLUE;
//    private StartPosition startPosition = StartPosition.FRONT;
//    private AutoPath autoPath = AutoPath.SCORE_AND_PARK;
//
//    private Gamepad.RumbleEffect customRumbleEffect;
//    private boolean dpad_up_pressed, dpad_down_pressed, dpad_left_pressed, dpad_right_pressed;
//
//    // ========== OPMODE MEMBERS ==========
//    private Follower follower;
//    private ElapsedTime pathTimer;
//    private int pathState;
//
//    // === PATHING ===
//    private Pose startPose, pickupFrontPose, pickupMiddlePose, pickupBackPose, scorePose, parkPose;
//    private PathChain scorePath;
//    private Path parkPath;
//    private PathChain cyclePath;
//
//    // ========== OpMode METHODS ==========
//
//    @Override
//    public void init() {
//        customRumbleEffect = new Gamepad.RumbleEffect.Builder().addStep(0.5, 0.5, 200).build();
//
//        // CORRECTED: Use the centralized initialization pattern
//        robot = new RobotHardwareContainer(hardwareMap, telemetry);
//        follower = Constants.createFollower(hardwareMap, robot);
//        robot.initTurret(follower, hardwareMap); // Init turret after follower is created
//
//        actionManager = new ActionManager(robot);
//        pathTimer = new ElapsedTime();
//
//        telemetry.addLine("Autonomous Configuration:");
//        telemetry.addLine("--------------------------------");
//        telemetry.addLine("Press D-Pad Left/Right for Alliance.");
//        telemetry.addLine("Press D-Pad Up/Down for Start Position.");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        if (gamepad1.dpad_left && !dpad_left_pressed) { alliance = GameState.Alliance.BLUE; gamepad1.runRumbleEffect(customRumbleEffect); }
//        else if (gamepad1.dpad_right && !dpad_right_pressed) { alliance = GameState.Alliance.RED; gamepad1.runRumbleEffect(customRumbleEffect); }
//
//        if (gamepad1.dpad_up && !dpad_up_pressed) { startPosition = StartPosition.FRONT; gamepad1.runRumbleEffect(customRumbleEffect); }
//        else if (gamepad1.dpad_down && !dpad_down_pressed) { startPosition = StartPosition.BACK; gamepad1.runRumbleEffect(customRumbleEffect); }
//
//        dpad_up_pressed = gamepad1.dpad_up;
//        dpad_down_pressed = gamepad1.dpad_down;
//        dpad_left_pressed = gamepad1.dpad_left;
//        dpad_right_pressed = gamepad1.dpad_right;
//
//        GameState.alliance = this.alliance; // Persist alliance choice to global state
//
//        telemetry.clearAll();
//        telemetry.addLine("--- Autonomous Configuration ---");
//        telemetry.addData("Alliance", alliance.toString());
//        telemetry.addData("Start Position", startPosition.toString());
//        telemetry.addLine("Press PLAY when ready.");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        // CORRECTED: Calibrate the turret at the start of the OpMode for safety.
//        robot.turret.calibrate();
//        calculatePoses();
//        buildPaths();
//        follower.setStartingPose(startPose);
//        setPathState(1);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        actionManager.update();
//        updatePath();
//
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("ActionManager State", actionManager.isBusy() ? "BUSY" : "IDLE");
//        telemetry.addData("X Position", follower.getPose().getX());
//        telemetry.addData("Y Position", follower.getPose().getY());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        if (follower != null) {
//            GameState.currentPose = follower.getPose(); // Persist final pose for TeleOp
//            follower.breakFollowing();
//        }
//        if (actionManager != null) actionManager.stopAll();
//    }
//
//    private void calculatePoses() {
//         if (alliance == GameState.Alliance.BLUE) {
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
//            scorePose = FieldPosePresets.BLUE_SCORE_CLOSE_TO_GOAL;
//            pickupFrontPose = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
//            pickupMiddlePose = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
//            pickupBackPose = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
//            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
//        } else { // RED Alliance
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
//            scorePose = FieldPosePresets.RED_SCORE_POSE;
//            pickupFrontPose = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
//            pickupMiddlePose = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
//            pickupBackPose = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
//            parkPose = FieldPosePresets.RED_AUTO_PARK;
//        }
//    }
//
//    private void buildPaths() {
//        Pose scoreControlPoint = new Pose(scorePose.getX(), 48, startPose.getHeading());
//        scorePath = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scoreControlPoint))
//                .addPath(new BezierLine(scoreControlPoint, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        parkPath = new Path(new BezierLine(scorePose, parkPose));
//        parkPath.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//
//        if (autoPath == AutoPath.CYCLE) {
//            Pose pickupControlPoint = new Pose(pickupFrontPose.getX(), 48, scorePose.getHeading());
//            cyclePath = follower.pathBuilder()
//                    .addPath(new BezierLine(scorePose, pickupControlPoint))
//                    .addPath(new BezierLine(pickupControlPoint, pickupFrontPose))
//                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickupFrontPose.getHeading())
//                    .build();
//        }
//    }
//
//    private void updatePath() {
//        switch (pathState) {
//            case 0: break; // IDLE
//            case 1: // Start first path (to score)
//                follower.followPath(scorePath);
//                setPathState(2);
//                break;
//            case 2: // Wait for score path to finish, then start the launch sequence
//                if (!follower.isBusy()) {
//                    actionManager.startGreenBallShoot(); // Use the ActionManager for the whole sequence
//                    setPathState(3);
//                }
//                break;
//            case 3: // Wait for the ActionManager to finish the entire launch sequence
//                if (!actionManager.isBusy()) {
//                    if (autoPath == AutoPath.SCORE_AND_PARK) {
//                        follower.followPath(parkPath);
//                        setPathState(10);
//                    } else { // CYCLE
//                        follower.followPath(cyclePath);
//                        setPathState(4);
//                    }
//                }
//                break;
//            case 4: // Wait for path to pickup to finish, then start intake
//                if (!follower.isBusy()) {
//                    actionManager.startIntake(); // Use the ActionManager for intake
//                    setPathState(5);
//                }
//                break;
//            case 5: // Wait for intake to finish
//                if (!actionManager.isBusy()) {
//                    setPathState(-1); // End of routine
//                }
//                break;
//            case 10: // Wait for park path to finish
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//            case -1: // STOP
//                follower.breakFollowing();
//                break;
//        }
//    }
//
//    private void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.reset();
//    }
//}
