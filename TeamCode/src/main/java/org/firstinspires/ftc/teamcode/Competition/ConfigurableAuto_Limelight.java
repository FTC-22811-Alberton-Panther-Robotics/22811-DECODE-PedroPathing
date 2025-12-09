package org.firstinspires.ftc.teamcode.InDevelopment;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.FieldPosePresets;
import org.firstinspires.ftc.teamcode.RobotHardware.GameState;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A highly configurable, command-based "playlist" autonomous routine.
 * <p>
 * This OpMode allows the drive team to build a sequence of commands during the
 * {@code init_loop()} phase, creating a custom autonomous routine on the fly.
 * This is extremely powerful for adapting to different alliance partners and match strategies.
 *
 * --- Init-Loop Controller Map ---
 * [GAMEPAD 1]
 * D-Pad Up/Down:     Select Alliance (BLUE/RED)
 * Left/Right Bumper: Select Start Position (FRONT/BACK)
 *
 * D-Pad Left/Right:  Scroll through the list of available {@link AutoCommand}s.
 * Button A:            Add the currently selected command to the end of the playlist.
 * Button B:            Remove the last command from the playlist.
 * Button X:            Finalize and lock the playlist.
 * Button Y:            Clear the entire playlist.
 *
 * --- Autonomous Commands Explained ---
 * - GO_TO_*_SPIKE: Drives the robot from its current position to the specified spike mark.
 *   This also sets the "context" for the INTAKE_CYCLE command.
 * - INTAKE_CYCLE: Performs the detailed, multi-step "intake dance" to pick up all three
 *   pixels from a spike mark line. It knows which line it's at based on the last
 *   `GO_TO` command.
 * - SCORE: Drives the robot to the appropriate scoring location and launches its artifacts.
 * - HIT_GATE: Drives through the gate to trigger the shared artifact drop.
 * - PARK: Drives the robot to a safe parking location.
 *
 * --- How it Works ---
 * During init, a list of commands (the playlist) is built. When the match starts,
 * a state machine (`updatePath`) iterates through the playlist, calling `executeCommand()`
 * for each one. Simple commands, like `PARK`, execute a single path. Complex commands,
 * like `INTAKE_CYCLE`, trigger their own multi-step sub-state machines.
 */
@Autonomous(name = "ConfigurableAuto Limelight", group = "01 Helena", preselectTeleOp = "TeleopManualControls")
public class ConfigurableAuto_Limelight extends OpMode {

    // ========== CONFIGURATION ==========
    private enum StartPosition { FRONT, BACK }

    private enum AutoCommand {
        GO_TO_FRONT_SPIKE,
        GO_TO_MIDDLE_SPIKE,
        GO_TO_BACK_SPIKE,
        INTAKE_CYCLE, // Perform the 3-pixel intake dance
        SCORE,
        HIT_GATE,
        PARK
    }

    // This enum is used to remember which spike mark we are at, which is crucial
    // for the INTAKE_CYCLE to know which color pattern to use.
    private enum SpikeLocation { NONE, FRONT, MIDDLE, BACK } 

    // --- Configurable Variables ---
    private GameState.Alliance alliance = GameState.Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    private SpikeLocation currentSpikeContext = SpikeLocation.NONE;
    private ArrayList<AutoCommand> autoCommands = new ArrayList<>();
    private int commandMenuIndex = 0;
    private int currentCommandIndex = 0;

    // --- Button Press Trackers ---
    private boolean dpad_up_down_pressed, dpad_left_right_pressed, bumper_pressed, a_pressed, y_pressed, x_pressed, b_pressed;
    private boolean playlistFinalized = false;

    // ========== OPMODE MEMBERS ==========
    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private RobotHardwareContainer robot;
    private ActionManager actionManager;

    // === PATHING & STATE ===
    private Pose startPose, scorePose, parkPose, frontSpike, middleSpike, backSpike, gateApproachPose, gateTriggerPose;
    private int pathState = 0;

    // --- Intake Cycle Specific Variables ---
    private int intakeCycleBallCount = 0;
    private List<Character> intakeColorOrder = new ArrayList<>();
    private final List<Character> SPIKE_FRONT_COLORS = Arrays.asList('G', 'P', 'P');
    private final List<Character> SPIKE_MIDDLE_COLORS = Arrays.asList('P', 'G', 'P');
    private final List<Character> SPIKE_BACK_COLORS = Arrays.asList('P', 'P', 'G');
    private final double INTAKE_OFFSET_DISTANCE = 6.0;


    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap, robot.localizer);

        telemetry.addLine("--- Playlist Autonomous Builder ---");
        telemetry.addLine("D-Pad U/D: Alliance | Bumpers: Start Pos");
        telemetry.addLine("D-Pad L/R: Select | A: Add | B: Remove Last");
        telemetry.addLine("X: Finalize/Lock | Y: Clear All");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (!playlistFinalized) {
            // --- Basic Configuration ---
            if (gamepad1.dpad_up && !dpad_up_down_pressed) alliance = GameState.Alliance.BLUE;
            if (gamepad1.dpad_down && !dpad_up_down_pressed) alliance = GameState.Alliance.RED;
            dpad_up_down_pressed = gamepad1.dpad_up || gamepad1.dpad_down;

            if (gamepad1.left_bumper && !bumper_pressed) startPosition = StartPosition.FRONT;
            if (gamepad1.right_bumper && !bumper_pressed) startPosition = StartPosition.BACK;
            bumper_pressed = gamepad1.left_bumper || gamepad1.right_bumper;

            GameState.alliance = this.alliance;

            // --- Playlist Builder Logic ---
            AutoCommand[] allCommands = AutoCommand.values();
            if (gamepad1.dpad_right && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex + 1) % allCommands.length;
            if (gamepad1.dpad_left && !dpad_left_right_pressed) commandMenuIndex = (commandMenuIndex - 1 + allCommands.length) % allCommands.length;
            dpad_left_right_pressed = gamepad1.dpad_left || gamepad1.dpad_right;

            if (gamepad1.a && !a_pressed) autoCommands.add(allCommands[commandMenuIndex]);
            a_pressed = gamepad1.a;

            // 'B' button removes the last command from the playlist.
            if (gamepad1.b && !b_pressed && !autoCommands.isEmpty()) {
                autoCommands.remove(autoCommands.size() - 1);
            }
            b_pressed = gamepad1.b;

            if (gamepad1.y && !y_pressed) autoCommands.clear();
            y_pressed = gamepad1.y;

            // 'X' button finalizes the playlist, preventing further edits.
            if (gamepad1.x && !x_pressed) {
                playlistFinalized = true;
            }
            x_pressed = gamepad1.x;
        }

        updateInitTelemetry(AutoCommand.values());
    }

    @Override
    public void start() {
        calculatePoses();
        follower.setStartingPose(startPose);
        currentCommandIndex = 0;
        if (!autoCommands.isEmpty()) {
            setPathState(1); // Start the playlist
        } else {
            setPathState(-1); // No commands, end immediately
        }
    }

    @Override
    public void stop() {
        // This method is called automatically when the autonomous period ends.
        // We save the robot's final position to the GameState class so that TeleOp
        // can know where the robot is on the field for a seamless transition.
        if (follower != null) {
            GameState.currentPose = follower.getPose();
        }
    }

    @Override
    public void loop() {
        follower.update();
        actionManager.update();
        updatePath();
        
        // Provide live feedback during the autonomous run.
        telemetry.addData("Executing Step", (currentCommandIndex + 1) + " of " + autoCommands.size());
        telemetry.addData("Command", (currentCommandIndex < autoCommands.size()) ? autoCommands.get(currentCommandIndex) : "DONE");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    /** Populates all the Pose variables with the correct coordinates based on the selected alliance. */
    private void calculatePoses() {
        if (GameState.alliance == GameState.Alliance.BLUE) {
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.BLUE_FRONT_START : FieldPosePresets.BLUE_BACK_START;
            scorePose = FieldPosePresets.BLUE_SCORE_POSE;
            parkPose = FieldPosePresets.BLUE_AUTO_PARK;
            frontSpike = FieldPosePresets.BLUE_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.BLUE_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.BLUE_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.BLUE_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.BLUE_GATE_TRIGGER;
        } else { // RED Alliance
            startPose = (startPosition == StartPosition.FRONT) ? FieldPosePresets.RED_FRONT_START : FieldPosePresets.RED_BACK_START;
            scorePose = FieldPosePresets.RED_SCORE_POSE;
            parkPose = FieldPosePresets.RED_AUTO_PARK;
            frontSpike = FieldPosePresets.RED_PICKUP_FRONT_SPIKE;
            middleSpike = FieldPosePresets.RED_PICKUP_MIDDLE_SPIKE;
            backSpike = FieldPosePresets.RED_PICKUP_BACK_SPIKE;
            gateApproachPose = FieldPosePresets.RED_GATE_APPROACH;
            gateTriggerPose = FieldPosePresets.RED_GATE_TRIGGER;
        }
    }

    /** Resets the state timer and updates the state. */
    private void setPathState(int newState) {
        pathState = newState;
        timer.reset();
    }

    /** The main state machine that executes the command playlist. */
    private void updatePath() {
        if (currentCommandIndex >= autoCommands.size()) {
            setPathState(-1); // Done with playlist
            return;
        }

        switch (pathState) {
            case 0: break; // IDLE

            case 1: executeCommand(autoCommands.get(currentCommandIndex)); break;

            case 2: // Generic "wait" state for simple path/action commands.
                if (!follower.isBusy() && !actionManager.isBusy()) advanceToNextCommand();
                break;

            // --- Scoring Sub-States (100-series) ---
            case 100: if (!follower.isBusy()) { actionManager.startLaunch(); setPathState(101); } break;
            case 101: if (!actionManager.isBusy()) advanceToNextCommand(); break;

            // --- Gate Hitting Sub-States (200-series) ---
            case 200: 
                if (!follower.isBusy()) { 
                    followPath(gateTriggerPose);
                    setPathState(201); 
                } 
                break;
            case 201: 
                if (!follower.isBusy()) { 
                    followPath(gateApproachPose);
                    setPathState(202);
                } 
                break;
            case 202: if (!follower.isBusy()) advanceToNextCommand(); break;

            // --- Intake Cycle Sub-States (300-series) ---
            case 300: // Start of the intake dance
                intakeCycleBallCount = 0; // Reset ball count
                setPathState(301);
                break;
            case 301: // Set diverter for the current ball
                if (!actionManager.isBusy()) {
                    char expectedColor = intakeColorOrder.get(intakeCycleBallCount);
                    if (expectedColor == 'P') actionManager.setDiverterPurple(); else actionManager.setDiverterGreen();
                    setPathState(302);
                }
                break;
            case 302: // Start timed intake action
                actionManager.startIntake();
                setPathState(303);
                break;
            case 303: // Wait for intake to finish
                if (!actionManager.isBusy()) {
                    intakeCycleBallCount++;
                    if (intakeCycleBallCount >= 3) { // Finished all 3 balls
                        advanceToNextCommand();
                    } else { // Move to the next ball
                        double offset = (alliance == GameState.Alliance.BLUE) ? -INTAKE_OFFSET_DISTANCE : INTAKE_OFFSET_DISTANCE;
                        Pose currentPose = follower.getPose();
                        Pose nextBallPose = currentPose.plus(new Pose(0, offset, 0));
                        // For the small strafe, we want to maintain our heading.
                        Path pathToNextBall = new Path(new BezierLine(currentPose, nextBallPose));
                        pathToNextBall.setConstantHeadingInterpolation(currentPose.getHeading());
                        follower.followPath(pathToNextBall);
                        setPathState(301); // Go back to set diverter for the next ball
                    }
                }
                break;

            case -1: // DONE
            default: follower.breakFollowing(); break;
        }
    }

    /** Reads the current command from the playlist and starts the appropriate action. */
    private void executeCommand(AutoCommand command) {
        switch (command) {
            case GO_TO_FRONT_SPIKE:
                currentSpikeContext = SpikeLocation.FRONT;
                followPath(frontSpike);
                setPathState(2);
                break;
            case GO_TO_MIDDLE_SPIKE:
                currentSpikeContext = SpikeLocation.MIDDLE;
                followPath(middleSpike);
                setPathState(2);
                break;
            case GO_TO_BACK_SPIKE:
                currentSpikeContext = SpikeLocation.BACK;
                followPath(backSpike);
                setPathState(2);
                break;
            case INTAKE_CYCLE:
                if (currentSpikeContext == SpikeLocation.FRONT) intakeColorOrder = SPIKE_FRONT_COLORS;
                else if (currentSpikeContext == SpikeLocation.MIDDLE) intakeColorOrder = SPIKE_MIDDLE_COLORS;
                else intakeColorOrder = SPIKE_BACK_COLORS;
                setPathState(300);
                break;
            case SCORE:
                followPath(scorePose);
                setPathState(100);
                break;
            case HIT_GATE:
                followPath(gateApproachPose);
                setPathState(200);
                break;
            case PARK:
                followPath(parkPose);
                setPathState(2);
                break;
        }
    }

    /** Small helper method to build a path with heading interpolation, keeping the call site clean. */
    private void followPath(Pose targetPose) {
        Pose currentPose = follower.getPose();
        Path newPath = new Path(new BezierLine(currentPose, targetPose));
        newPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading());
        follower.followPath(newPath);
    }

    /** Advances the playlist to the next command. */
    private void advanceToNextCommand() {
        currentCommandIndex++;
        setPathState(1); // Go execute the next command (or finish if done)
    }

    /** Updates the telemetry during the init_loop with the current configuration. */
    private void updateInitTelemetry(AutoCommand[] allCommands) {
        telemetry.addData("Alliance", GameState.alliance).addData("Start", startPosition);
        telemetry.addLine("\n--- Build Your Playlist ---");
        if (playlistFinalized) {
            telemetry.addLine("\nPLAYLIST FINALIZED!");
        } else {
            telemetry.addData("--> Selected Command", allCommands[commandMenuIndex]);
        }
        telemetry.addLine("\nCurrent Playlist:");
        if (autoCommands.isEmpty()) {
            telemetry.addLine("  (empty)");
        } else {
            for (int i = 0; i < autoCommands.size(); i++) {
                telemetry.addLine((i + 1) + ". " + autoCommands.get(i));
            }
        }
        telemetry.update();
    }
}
