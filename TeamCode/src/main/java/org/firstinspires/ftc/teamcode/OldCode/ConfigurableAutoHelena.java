//package org.firstinspires.ftc.teamcode.OldCode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.geometry.BezierLine;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
//import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
//import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
//import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//@Autonomous(name = "ConfigurableAuto Test", group = "01 Helena", preselectTeleOp = "HelenaTeleOp_Test")
//public class ConfigurableAutoHelena extends OpMode {
//
//    // ========== CONFIGURATION ==========
//    private enum StartPosition { FRONT, BACK }
//
//    private enum AutoCommand {
//        GO_TO_FRONT_SPIKE,
//        GO_TO_MIDDLE_SPIKE,
//        GO_TO_BACK_SPIKE,
//        INTAKE_CYCLE,
//        SCORE,
//        HIT_GATE,
//        PARK
//    }
//
//    private enum SpikeLocation { NONE, FRONT, MIDDLE, BACK }
//
//    // --- Configurable Variables ---
//    private GameState.Alliance alliance = GameState.Alliance.BLUE;
//    private StartPosition startPosition = StartPosition.FRONT;
//    private SpikeLocation currentSpikeContext = SpikeLocation.NONE;
//    private ArrayList<AutoCommand> autoCommands = new ArrayList<>();
//    private int commandMenuIndex = 0;
//    private int currentCommandIndex = 0;
//
//    // --- Button Press Trackers ---
//    private boolean dpad_up_down_pressed, dpad_left_right_pressed, bumper_pressed, a_pressed, y_pressed, x_pressed, b_pressed;
//    private boolean playlistFinalized = false;
//
//    // ========== OPMODE MEMBERS ==========
//    private Follower follower;
//    private ElapsedTime timer = new ElapsedTime();
//    private RobotHardwareContainer robot;
//    private ActionManager actionManager;
//
//    // === PATHING & STATE ===
//    private Pose startPose, scorePose, parkPose, frontSpike, middleSpike, backSpike, gateApproachPose, gateTriggerPose;
//    private int pathState = 0;
//
//    // --- Intake Cycle Specific Variables ---
//    private int intakeCycleBallCount = 0;
//    private List<Character> intakeColorOrder = new ArrayList<>();
//    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
//    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
//    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
//    private final double INTAKE_OFFSET_DISTANCE = 6.0;
//
//
//    @Override
//    public void init() {
//        robot = new RobotHardwareContainer(hardwareMap, telemetry);
//        follower = Constants.createFollower(hardwareMap, robot);
//        robot.initTurret(follower, hardwareMap);
//        robot.initLauncher(follower, robot.turret, hardwareMap);
//        actionManager = new ActionManager(robot);
//
//        telemetry.addLine("--- Playlist Autonomous Builder (TEST) ---");
//        telemetry.addLine("D-Pad U/D: Alliance | Bumpers: Start Pos");
//        telemetry.addLine("D-Pad L/R: Select | A: Add | B: Remove Last");
//        telemetry.addLine("X: Finalize/Lock | Y: Clear All");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        if (!playlistFinalized) {
//            // --- Basic Configuration ---
//            if (gamepad1.dpad_up && !dpad_up_down_pressed) alliance = GameState.Alliance.BLUE;
//            if (gamepad1.dpad_down && !dpad_up_down_pressed) alliance = GameState.Alliance.RED;
//            dpad_up_down_pressed = gamepad1.dpad_up || gamepad1.dpad_down;
//
//            if (gamepad1.left_bumper && !bumper_pressed) startPosition = StartPosition.FRONT;
//            if (gamepad1.right_bumper && !bumper_pressed) startPosition = StartPosition.BACK;
//            bumper_pressed = gamepad1.left_bumper || gamepad1.right_bumper;
//
//            GameState.alliance = this.alliance;
//
//            // --- Playlist Builder Logic ---
//            AutoCommand[] allCommands = AutoCommand.values();
//            if (gamepad1.dpad_right && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex + 1) % allCommands.length;
//            if (gamepad1.dpad_left && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex - 1 + allCommands.length) % allCommands.length;
//            dpad_left_right_pressed = gamepad1.dpad_left || gamepad1.dpad_right;
//
//            if (gamepad1.a && !a_pressed) autoCommands.add(allCommands[commandMenuIndex]);
//            a_pressed = gamepad1.a;
//
//            if (gamepad1.b && !b_pressed && !autoCommands.isEmpty()) autoCommands.remove(autoCommands.size() - 1);
//            b_pressed = gamepad1.b;
//
//            if (gamepad1.y && !y_pressed) autoCommands.clear();
//            y_pressed = gamepad1.y;
//
//            if (gamepad1.x && !x_pressed) playlistFinalized = true;
//            x_pressed = gamepad1.x;
//        }
//
//        updateInitTelemetry(AutoCommand.values());
//    }
//
//    @Override
//    public void start() {
//        robot.turret.calibrate();
//        calculatePoses();
//        follower.setStartingPose(startPose);
//        currentCommandIndex = 0;
//        if (!autoCommands.isEmpty()) setPathState(1); else setPathState(-1);
//    }
//
//    @Override
//    public void stop() {
//        if (follower != null) GameState.currentPose = follower.getPose();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        actionManager.update();
//        robot.turret.update(alliance);
//        robot.launcher.update();
//        updatePath();
//        updateTelemetry();
//    }
//
//    private void calculatePoses() {
//        if (GameState.alliance == GameState.Alliance.BLUE) {
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
//            scorePose = FieldPosePresets.BLUE_SCORE_CLOSE_TO_GOAL;
//            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
//            frontSpike = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
//            middleSpike = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
//            backSpike = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
//            gateApproachPose = FieldPosePresets.BLUE_GATE_APPROACH;
//            gateTriggerPose = FieldPosePresets.BLUE_GATE_TRIGGER;
//        } else { // RED Alliance
//            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
//            scorePose = FieldPosePresets.RED_SCORE_POSE;
//            parkPose = FieldPosePresets.RED_AUTO_PARK;
//            frontSpike = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
//            middleSpike = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
//            backSpike = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
//            gateApproachPose = FieldPosePresets.RED_GATE_APPROACH;
//            gateTriggerPose = FieldPosePresets.RED_GATE_TRIGGER;
//        }
//    }
//
//    private void setPathState(int newState) {
//        pathState = newState;
//        timer.reset();
//    }
//
//    private void updatePath() {
//        if (currentCommandIndex >= autoCommands.size()) {
//            setPathState(-1);
//        }
//
//        switch (pathState) {
//            case 0: break; // IDLE
//            case 1: executeCommand(autoCommands.get(currentCommandIndex)); break;
//            case 2: if (!follower.isBusy() && !actionManager.isBusy()) advanceToNextCommand(); break;
//
//            // Scoring Sub-States (100-series)
//            case 100: if (!follower.isBusy()) { actionManager.startSmartLaunch(GameState.obeliskPattern); setPathState(101); } break;
//            case 101: if (!actionManager.isBusy()) advanceToNextCommand(); break;
//
//            // Gate Hitting Sub-States (200-series)
//            case 200: if (!follower.isBusy()) { followPath(gateTriggerPose); setPathState(201); } break;
//            case 201: if (!follower.isBusy()) { followPath(gateApproachPose); setPathState(202); } break;
//            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;
//
//            // Intake Cycle Sub-States (300-series)
//            case 300: intakeCycleBallCount = 0; setPathState(301); break;
//            case 301: if (!actionManager.isBusy()) { setDiverterForIntake(); setPathState(302); } break;
//            case 302: actionManager.startIntake(); setPathState(303); break;
//            case 303: if (!actionManager.isBusy()) { intakeCycleBallCount++; if (intakeCycleBallCount >= 3) { advanceToNextCommand(); } else { moveToNextPixel(); setPathState(301); } } break;
//
//            case -1: default: follower.breakFollowing(); break;
//        }
//    }
//
//    private void executeCommand(AutoCommand command) {
//        switch (command) {
//            case GO_TO_FRONT_SPIKE: currentSpikeContext = SpikeLocation.FRONT; followPath(frontSpike); setPathState(2); break;
//            case GO_TO_MIDDLE_SPIKE: currentSpikeContext = SpikeLocation.MIDDLE; followPath(middleSpike); setPathState(2); break;
//            case GO_TO_BACK_SPIKE: currentSpikeContext = SpikeLocation.BACK; followPath(backSpike); setPathState(2); break;
//            case INTAKE_CYCLE: setIntakeColorOrder(); setPathState(300); break;
//            case SCORE:
//                if (!robot.launcher.isLauncherOn()) {
//                    robot.launcher.toggleLauncher();
//                }
//                followPath(scorePose);
//                setPathState(100);
//                break;
//            case HIT_GATE: followPath(gateApproachPose); setPathState(200); break;
//            case PARK: followPath(parkPose); setPathState(2); break;
//        }
//    }
//
//    private void advanceToNextCommand() {
//        currentCommandIndex++;
//        setPathState(1);
//    }
//
//    private void followPath(Pose targetPose) {
//        Pose currentPose = follower.getPose();
//        if (currentPose == null) {
//            currentPose = this.startPose;
//        }
//        follower.followPath(new Path(new BezierLine(currentPose, targetPose)));
//    }
//
//    private void setDiverterForIntake() {
//        char expectedColor = intakeColorOrder.get(intakeCycleBallCount);
//        if (expectedColor == 'P') actionManager.setDiverterToPurple(); else actionManager.setDiverterToGreen();
//    }
//
//    private void moveToNextPixel() {
//        double offset = (alliance == GameState.Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
//        Pose nextBallPose = follower.getPose().plus(new Pose(0, offset, 0));
//        followPath(nextBallPose);
//    }
//
//    private void setIntakeColorOrder() {
//        if (currentSpikeContext == SpikeLocation.FRONT) intakeColorOrder = SPIKE_FRONT_COLORS;
//        else if (currentSpikeContext == SpikeLocation.MIDDLE) intakeColorOrder = SPIKE_MIDDLE_COLORS;
//        else intakeColorOrder = SPIKE_BACK_COLORS;
//    }
//
//    private void updateInitTelemetry(AutoCommand[] allCommands) {
//        telemetry.clearAll();
//        telemetry.addLine("--- Playlist Autonomous Builder (TEST) ---");
//        telemetry.addData("Alliance", alliance).addData("Start", startPosition);
//        telemetry.addLine();
//        telemetry.addLine("SELECTED: [" + allCommands[commandMenuIndex] + "] (Press A to Add)");
//        telemetry.addLine("-----------------------------------");
//        telemetry.addLine("Current Playlist: (" + autoCommands.size() + " steps)");
//        for (int i = 0; i < autoCommands.size(); i++) {
//            telemetry.addLine((i + 1) + ". " + autoCommands.get(i));
//        }
//        if (playlistFinalized) {
//            telemetry.addLine("\n*** PLAYLIST FINALIZED ***");
//        }
//        telemetry.update();
//    }
//
//    private void updateTelemetry() {
//        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
//        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("Obelisk Pattern", GameState.obeliskPattern);
//        telemetry.addData("Turret Current (A)", "%.2f", robot.turret.getTurretCurrent());
//        telemetry.addData("Intake Current (A)", "%.2f", robot.intake.getIntakeCurrent());
//        telemetry.update();
//    }
//}
