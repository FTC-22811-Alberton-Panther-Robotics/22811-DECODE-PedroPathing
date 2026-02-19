package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.DiverterHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A highly configurable, playlist-based Autonomous OpMode with combined actions.
 *
 * This OpMode allows the drive team to build a sequence of autonomous actions. Key commands
 * are combined to make playlist creation faster and more intuitive. For example, selecting
 * a spike mark command will automatically handle both moving to the spike and intaking.
 *
 * --- KEY FEATURES ---
 * - Combined Actions: Complex multi-step actions (e.g., "go to spike and intake all three")
 *   are condensed into single, selectable commands.
 * - State Persistence: Saves the robot's final pose and alliance to the static `GameState`,
 *   allowing `BozemanTeleop` to seamlessly take over with full field awareness.
 *
 * --- PLAYLIST BUILDER CONTROLS (GAMEPAD 1 - INIT) ---
 * D-Pad Up/Down:   Select Alliance (Blue/Red)
 * Left/Right Bumper: Select Starting Position (Front/Back)
 * D-Pad Left/Right:  Scroll through the list of available auto commands.
 * A Button:          Add the currently selected command to the playlist.
 * B Button:          Remove the last command from the playlist.
 * Y Button:          Clear the entire playlist.
 * X Button:          Finalize/Lock the playlist to prevent accidental changes.
 *
 */
@Autonomous(name = "Bozeman Auto", group = "01 Bozeman", preselectTeleOp = "BozemanTeleop")
public class BozemanAuto extends OpMode {

    // ========== OPMODE CONFIGURATION (SET DURING INIT) ========== //

    private enum StartPosition { FRONT, BACK }

    private enum AutoCommand {
        SCORE_ALL_THREE_CLOSE,
        SCORE_ALL_THREE_FAR,
        SPIKE_FRONT_AND_INTAKE,
        SPIKE_MIDDLE_AND_INTAKE,
        SPIKE_BACK_AND_INTAKE,
        HIT_GATE,
        PARK
    }
    private enum SpikeLocation { NONE, FRONT, MIDDLE, BACK }

    private GameState.Alliance alliance = GameState.Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    private SpikeLocation currentSpikeContext = SpikeLocation.NONE;

    private ArrayList<AutoCommand> autoCommands = new ArrayList<>();
    private int commandMenuIndex = 0;
    private int currentCommandIndex = 0;
    private boolean isStartPoseSelected = false;
    private boolean isPlaylistFinalized = false;

    // ========== OPMODE CORE MEMBERS ========== //
    private Localizer localizer; // Now correctly assigned from the Follower
    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private RobotHardwareContainer robot;
    private ActionManager actionManager;

    // === PATHING & STATE MANAGEMENT === //
    private Pose startPose, scoreClosePose, scoreFarPose, parkPose;
    private Pose frontSpike, middleSpike, backSpike;
    private Pose gateApproachPose, gateTriggerPose;
    private int pathState = 0;

    private int intakeCycleArtifactCount = 0;
    private int scoreCycleArtifactCount = 0;
    private List<Character> intakeColorOrder = new ArrayList<>();
    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
    private final double INTAKE_OFFSET_DISTANCE = 6.0;
    private final double INTAKE_PAUSE_SECONDS = 0.5; // TODO: Tune this pause time

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);

        // --- HARDWARE FIX: Single Source of Truth for Localizer ---
        // The FollowerBuilder is now the single source of truth for creating the PinpointLocalizer.
        // We do NOT create a separate PinpointLocalizer here. Instead, we create the Follower
        // and then get the single, authoritative localizer instance FROM it.
        follower = Constants.createFollower(hardwareMap);

        robot.initTurret(follower, hardwareMap);
        robot.initLauncher(follower, hardwareMap);

        telemetry.addLine("--- Playlist Autonomous Builder ---");
        telemetry.addLine("X: Lock Playlist | A: Add | B: Remove | Y: Clear");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!isStartPoseSelected) {
            selectStartingLocation();
        } else {
            playlistBuilder();
        }
    }

    @Override
    public void start() {
        GameState.alliance = this.alliance;
        robot.turret.calibrate();
        robot.intake.run();
        robot.launcher.start();

        calculatePoses();
        follower.setStartingPose(startPose);
        currentCommandIndex = 0;
        if (!autoCommands.isEmpty()) {
            setPathState(1);
        } else {
            setPathState(-1);
        }
        follower.update();
    }

    @Override
    public void loop() {
        // The Follower's update now handles the Localizer update internally.
        follower.update();
        actionManager.update();
        robot.turret.update(alliance);
        robot.launcher.update(alliance);

        updatePath();

        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Pose", "X: %.2f, Y: %.2f, H: %.1f", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower.getPose() != null) {
            GameState.currentPose = follower.getPose();
        }
    }

    private void calculatePoses() {
        if (alliance == GameState.Alliance.BLUE) {
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
            scoreClosePose = FieldPosePresets.BLUE_SCORE_CLOSE_TO_GOAL;
            scoreFarPose = FieldPosePresets.BLUE_SCORE_FAR_FROM_GOAL;
            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
            frontSpike = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.BLUE_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.BLUE_GATE_TRIGGER;
        } else { // RED Alliance
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
            scoreClosePose = FieldPosePresets.RED_SCORE_CLOSE_TO_GOAL;
            scoreFarPose = FieldPosePresets.RED_SCORE_FAR_FROM_GOAL;
            parkPose = FieldPosePresets.RED_AUTO_PARK;
            frontSpike = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.RED_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.RED_GATE_TRIGGER;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        timer.reset();
    }

    private void updatePath() {
        if (currentCommandIndex >= autoCommands.size()) {
            setPathState(-1);
            return;
        }

        switch (pathState) {
            case 0: break; // IDLE

            case 1: executeCommand(autoCommands.get(currentCommandIndex)); break;

            case 2: // A simple waiting state for basic paths or actions to complete.
                if (!follower.isBusy() && !actionManager.isBusy()) advanceToNextCommand();
                break;

            // --- Gate Hitting Sub-States ---
            case 200: if (!follower.isBusy()) { followerPathBuilder(gateTriggerPose); setPathState(201); } break;
            case 201: if (!follower.isBusy()) { followerPathBuilder(gateApproachPose); setPathState(202); } break;
            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;

            // --- Intake Cycle Sub-States (from working example) ---
            case 300: // Wait for arrival at the first spike mark.
                if (!follower.isBusy()) {
                    intakeCycleArtifactCount = 0; // Reset for the new cycle
                    setPathState(301);
                }
                break;
            case 301: // Set diverter and prepare for timed intake.
                if (intakeCycleArtifactCount == 0) {
                    robot.diverter.setPosition(DiverterHardware.GatePosition.GREEN);
                } else {
                    robot.diverter.setPosition(DiverterHardware.GatePosition.PURPLE);
                }
                robot.intake.run();
                setPathState(302); // Move to the waiting state, which resets the timer.
                break;
            case 302: // Wait for the artifact to be fully captured.
                if (timer.seconds() > 0.67) {
                    intakeCycleArtifactCount++;
                    setPathState(303);
                }
                break;
            case 303: // Check if we're done, or move to the next artifact.
                if (intakeCycleArtifactCount >= 3) {
                    advanceToNextCommand(); // We have all three, command is done.
                } else {
                    double offset = (alliance == GameState.Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
                    Pose nextArtifactPose = follower.getPose().plus(new Pose(offset, 0, 0));
                    followerPathBuilder(nextArtifactPose);
                    setPathState(304); // Go to a state that waits for the move to finish.
                }
                break;
            case 304: // Wait for the robot to finish moving to the next artifact.
                if (!follower.isBusy()) {
                    setPathState(301); // Once there, go back to intake the next one.
                }
                break;

            // --- Scoring Cycle Sub-States ---
            case 400: // Wait for arrival at the scoring position.
                if (!follower.isBusy()) {
                    scoreCycleArtifactCount = 0; // Reset counter
                    setPathState(401);
                }
                break;
            case 401: // Fire all three artifacts in the specified order.
                if (!actionManager.isBusy()) {
                    if (scoreCycleArtifactCount < intakeColorOrder.size()) {
                        char launchSide = intakeColorOrder.get(scoreCycleArtifactCount);
                        actionManager.autonomousScore(launchSide);
                        scoreCycleArtifactCount++;
                    } else {
                        advanceToNextCommand(); // All three have been fired, move on.
                    }
                }
                break;

            case -1: // DONE
            default: follower.breakFollowing(); break;
        }
    }

    private void executeCommand(AutoCommand command) {
        switch (command) {
            case SPIKE_FRONT_AND_INTAKE:
                currentSpikeContext = SpikeLocation.FRONT;
                intakeColorOrder = SPIKE_FRONT_COLORS;
                followerPathBuilder(frontSpike);
                setPathState(300);
                break;
            case SPIKE_MIDDLE_AND_INTAKE:
                currentSpikeContext = SpikeLocation.MIDDLE;
                intakeColorOrder = SPIKE_MIDDLE_COLORS;
                followerPathBuilder(middleSpike);
                setPathState(300);
                break;
            case SPIKE_BACK_AND_INTAKE:
                currentSpikeContext = SpikeLocation.BACK;
                intakeColorOrder = SPIKE_BACK_COLORS;
                followerPathBuilder(backSpike);
                setPathState(300);
                break;
            case SCORE_ALL_THREE_CLOSE:
                followerPathBuilder(scoreClosePose);
                setPathState(400);
                break;
            case SCORE_ALL_THREE_FAR:
                followerPathBuilder(scoreFarPose);
                setPathState(400);
                break;
            case HIT_GATE:
                followerPathBuilder(gateApproachPose);
                setPathState(200);
                break;
            case PARK:
                followerPathBuilder(parkPose);
                setPathState(2);
                break;
        }
    }

    private void advanceToNextCommand() {
        currentCommandIndex++;
        setPathState(1);
    }

    private void followerPathBuilder(Pose targetPose){
        Path pathToFollow = new Path(new BezierLine(follower.getPose(), targetPose));
        pathToFollow.setLinearHeadingInterpolation(follower.getPose().getHeading(), targetPose.getHeading());
        follower.followPath(pathToFollow);
    }

    private void selectStartingLocation(){
        if (gamepad1.dpadLeftWasPressed()) alliance = GameState.Alliance.BLUE;
        if (gamepad1.dpadRightWasPressed()) alliance = GameState.Alliance.RED;
        if (gamepad1.dpadUpWasPressed()) startPosition = StartPosition.BACK;
        if (gamepad1.dpadDownWasPressed()) startPosition = StartPosition.FRONT;

        if (gamepad1.a) {
            isStartPoseSelected = true;
        }

        telemetry.addLine("Blue/Red Alliance: dPad left/right");
        telemetry.addLine("Starting Position Back/Front: dPad up/down");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Start", startPosition);
        telemetry.addLine("\nPress 'A' to confirm and start building playlist");
        telemetry.update();
    }

    private void playlistBuilder() {
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Start", startPosition);

        if (gamepad1.x) {
            isPlaylistFinalized = true;
        } else if (gamepad1.back) {
            isPlaylistFinalized = false;
        }

        if (!isPlaylistFinalized) {
            AutoCommand[] allCommands = AutoCommand.values();
            if (gamepad1.dpadRightWasPressed()) commandMenuIndex = (commandMenuIndex + 1) % allCommands.length;
            if (gamepad1.dpadLeftWasPressed()) commandMenuIndex = (commandMenuIndex - 1 + allCommands.length) % allCommands.length;

            if (gamepad1.aWasPressed()) autoCommands.add(allCommands[commandMenuIndex]);
            if (gamepad1.yWasPressed()) autoCommands.clear();
            if (gamepad1.bWasPressed() && !autoCommands.isEmpty()) {
                autoCommands.remove(autoCommands.size() - 1);
            }
        }

        if (isPlaylistFinalized) {
            telemetry.addLine("*** PLAYLIST FINALIZED (Press BACK to Unlock) ***");
        } else {
            telemetry.addLine("\nA: add command, B: remove last, Y: clear, X: finalize");
            telemetry.addData("--> Selected Command", AutoCommand.values()[commandMenuIndex]);
        }

        telemetry.addLine("\nCurrent Playlist:");
        for (int i = 0; i < autoCommands.size(); i++) {
            telemetry.addLine((i + 1) + ". " + autoCommands.get(i));
        }
        telemetry.update();
    }
}
