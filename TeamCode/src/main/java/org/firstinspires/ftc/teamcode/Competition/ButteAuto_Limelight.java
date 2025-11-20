package org.firstinspires.ftc.teamcode.Competition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware.ActionManager;
import org.firstinspires.ftc.teamcode.RobotHardware.LimelightAprilTagLocalizer;
import org.firstinspires.ftc.teamcode.RobotHardware.RobotHardwareContainer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Optional;

@Autonomous(name = "ButteAuto (Limelight)", group = "01 Butte")
public class ButteAuto_Limelight extends OpMode {

    // ========== CONFIGURATION ==========
    private enum Alliance { BLUE, RED }
    private enum StartPosition { FRONT, BACK }
    private enum SpikeLocation { LEFT, MIDDLE, RIGHT, UNKNOWN }

    private Alliance alliance = Alliance.BLUE;
    private StartPosition startPosition = StartPosition.FRONT;
    private SpikeLocation spikeLocation = SpikeLocation.UNKNOWN;

    private boolean dpad_up_down_pressed = false;
    private boolean bumper_pressed = false;

    // ========== OPMODE MEMBERS ==========
    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private RobotHardwareContainer robot;
    private ActionManager actionManager;

    // === PATHING POSES ===
    private Pose startPose, launchPose, spikePose, parkPose;

    // === PATHS ===
    private Path preloadsPath, intakePath, cycleLaunchPath, parkPath;

    private int pathState;

    @Override
    public void init() {
        robot = new RobotHardwareContainer(hardwareMap, telemetry);
        actionManager = new ActionManager(robot);
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("--- Limelight Auto Configuration ---");
        telemetry.addLine("D-Pad Up/Down: Change Alliance (BLUE/RED)");
        telemetry.addLine("Left/Right Bumper: Change Start (FRONT/BACK)");
        telemetry.addLine("Point camera at AprilTag to detect Spike Mark.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // --- Alliance and Start Position Selection ---
        if (gamepad1.dpad_up && !dpad_up_down_pressed) alliance = Alliance.BLUE;
        if (gamepad1.dpad_down && !dpad_up_down_pressed) alliance = Alliance.RED;
        dpad_up_down_pressed = gamepad1.dpad_up || gamepad1.dpad_down;

        if (gamepad1.left_bumper && !bumper_pressed) startPosition = StartPosition.FRONT;
        if (gamepad1.right_bumper && !bumper_pressed) startPosition = StartPosition.BACK;
        bumper_pressed = gamepad1.left_bumper || gamepad1.right_bumper;

        // --- AprilTag Detection ---
        Optional<Integer> detectedIdOptional = robot.aprilTag.getDetectedTagId();
        if (detectedIdOptional.isPresent()) {
            int detectedId = detectedIdOptional.get();
            if (detectedId == LimelightAprilTagLocalizer.MOTIF_GPP_ID) {
                spikeLocation = SpikeLocation.LEFT;
            } else if (detectedId == LimelightAprilTagLocalizer.MOTIF_PGP_ID) {
                spikeLocation = SpikeLocation.MIDDLE;
            } else if (detectedId == LimelightAprilTagLocalizer.MOTIF_PPG_ID) {
                spikeLocation = SpikeLocation.RIGHT;
            }
        }

        telemetry.addData("Alliance", alliance.toString());
        telemetry.addData("Start Position", startPosition.toString());
        telemetry.addData("Detected Spike", spikeLocation.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.aprilTag.start();
        calculatePoses();
        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(1);
    }

    @Override
    public void loop() {
        follower.update();
        actionManager.update();
        updatePath();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionManager.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.aprilTag.stop();
        actionManager.stopAll();
        follower.breakFollowing();
    }

    private void calculatePoses() {
        // Poses for BLUE_FRONT scenario, to be mirrored. TUNE THESE FOR YOUR ROBOT!
        Pose blueFrontStart = new Pose(12, 12, Math.toRadians(90));
        Pose blueFrontLaunch = new Pose(36, 48, Math.toRadians(45)); // Position to launch pre-loads
        Pose blueFrontSpikeLeft = new Pose(12, 84, Math.toRadians(135));
        Pose blueFrontSpikeMiddle = new Pose(24, 90, Math.toRadians(90));
        Pose blueFrontSpikeRight = new Pose(36, 84, Math.toRadians(45));
        Pose blueFrontPark = new Pose(60, 12, Math.toRadians(0)); // Park outside launch zone

        // Logic to select correct poses based on configuration
        if (alliance == Alliance.BLUE && startPosition == StartPosition.FRONT) {
            startPose = blueFrontStart;
            launchPose = blueFrontLaunch;
            parkPose = blueFrontPark;
            if (spikeLocation == SpikeLocation.LEFT) spikePose = blueFrontSpikeLeft;
            else if (spikeLocation == SpikeLocation.MIDDLE) spikePose = blueFrontSpikeMiddle;
            else spikePose = blueFrontSpikeRight;
        } else if (alliance == Alliance.RED && startPosition == StartPosition.FRONT) {
            // Mirror logic for RED_FRONT
            startPose = new Pose(144 - blueFrontStart.getX(), blueFrontStart.getY(), Math.toRadians(-90));
            launchPose = new Pose(144 - blueFrontLaunch.getX(), blueFrontLaunch.getY(), Math.toRadians(-45));
            parkPose = new Pose(144 - blueFrontPark.getX(), blueFrontPark.getY(), Math.toRadians(180));

            // Spike marks are mirrored for RED
            if (spikeLocation == SpikeLocation.LEFT)
                spikePose = new Pose(144 - blueFrontSpikeRight.getX(), blueFrontSpikeRight.getY(), Math.toRadians(-45));
            else if (spikeLocation == SpikeLocation.MIDDLE)
                spikePose = new Pose(144 - blueFrontSpikeMiddle.getX(), blueFrontSpikeMiddle.getY(), Math.toRadians(-90));
            else
                spikePose = new Pose(144 - blueFrontSpikeLeft.getX(), blueFrontSpikeLeft.getY(), Math.toRadians(-135));
        } else {
            // Default to BLUE_FRONT if a combo isn't defined (e.g., BACK positions)
            startPose = blueFrontStart;
            launchPose = blueFrontLaunch;
            parkPose = blueFrontPark;
            spikePose = blueFrontSpikeMiddle;
        }
    }

    private void buildPaths() {
        // Path 1: From start to the first launch position
        preloadsPath = new Path(new BezierLine(startPose, launchPose));
        preloadsPath.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        // Path 2: From launch position to the correct spike mark for intake
        intakePath = new Path(new BezierLine(launchPose, spikePose));
        intakePath.setLinearHeadingInterpolation(launchPose.getHeading(), spikePose.getHeading());

        // Path 3: From the spike mark back to the launch position
        cycleLaunchPath = new Path(new BezierLine(spikePose, launchPose));
        cycleLaunchPath.setLinearHeadingInterpolation(spikePose.getHeading(), launchPose.getHeading());

        // Path 4: From the final launch position to parking
        parkPath = new Path(new BezierLine(launchPose, parkPose));
        parkPath.setLinearHeadingInterpolation(launchPose.getHeading(), parkPose.getHeading());
    }

    private void setPathState(int newState) {
        pathState = newState;
        if(pathTimer != null) pathTimer.reset();
    }

    private void updatePath() {
        // This state machine follows the corrected "launch -> intake -> launch -> park" logic
        switch (pathState) {
            case 0: // IDLE
                break;

            case 1: // Drive to launch position
                follower.followPath(preloadsPath);
                setPathState(2);
                break;

            case 2: // Wait for path, then launch pre-loads
                if (!follower.isBusy()) {
                    actionManager.startLaunch();
                    setPathState(3);
                }
                break;

            case 3: // Wait for launch, then drive to intake
                if (!actionManager.isBusy()) {
                    actionManager.completeAction();
                    follower.followPath(intakePath);
                    setPathState(4);
                }
                break;

            case 4: // Wait for path, then intake artifacts
                if (!follower.isBusy()) {
                    actionManager.startIntake();
                    setPathState(5);
                }
                break;

            case 5: // Wait for intake, then drive back to launch
                if (!actionManager.isBusy()) {
                    actionManager.completeAction();
                    follower.followPath(cycleLaunchPath);
                    setPathState(6);
                }
                break;

            case 6: // Wait for path, then launch cycled artifacts
                if (!follower.isBusy()) {
                    actionManager.startLaunch();
                    setPathState(7);
                }
                break;

            case 7: // Wait for launch, then park
                if (!actionManager.isBusy()) {
                    actionManager.completeAction();
                    follower.followPath(parkPath);
                    setPathState(8);
                }
                break;

            case 8: // Wait for park path to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // End of routine
                }
                break;

            case -1:
            default:
                follower.breakFollowing();
                break;
        }
    }
}
