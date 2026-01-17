package org.firstinspires.ftc.teamcode.RobotHardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretHardware {

    public DcMotorEx turretMotor = null;
    public DcMotorEx leftFlywheel = null;
    public DcMotorEx rightFlywheel = null;
    private final double TURRET_GEAR_RATIO = 23.0/80;
    private final double TURRET_TICKS_PER_DEGREE = ((1 + (46.0 /11)) * 28)/360 * TURRET_GEAR_RATIO;
    private final Follower follower;
    public boolean validTargetSolution = false;
    public TurretHardware(Follower follower) {
        this.follower = follower;

    }


    public void init(HardwareMap hardwareMap){
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftFlywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightFlywheel");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);/// FIND ACTUAL DIRECTION
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stop();
    }

    ///  ** shooter control ** \\
    public void shooterWoundUp(){
        leftFlywheel.setVelocity(2400);
        rightFlywheel.setVelocity(2400);
    }
    /// ** turret control ** \\\
    public void rightSpin(double power){
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(power);
    }
    public void leftSpin(double power){
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(-power);
    }

    public void autoAim(GameState.Alliance alliance){
        Pose robotPose = follower.getPose();
        double robotHeading = robotPose.getHeading();

        // In target-lock mode, translation is field-centric.
        double sin_tl = Math.sin(-robotHeading);
        double cos_tl = Math.cos(-robotHeading);

        // But the robot's rotation is automatically handled by a P-controller to point at the goal.
        double headingError = MathFunctions.getSmallestAngleDifference(calculateHeadingToGoal(alliance), robotHeading);
        if (headingError > Math.PI){
            headingError = Math.PI;
            validTargetSolution = false;
        } else if (headingError < -Math.PI){
            headingError = -Math.PI;
            validTargetSolution = false;
        } else validTargetSolution = true;
        int turretTargetPosition = (int) (headingError * Math.toRadians(TURRET_TICKS_PER_DEGREE) - Math.PI/2);

        turretMotor.setTargetPosition(turretTargetPosition);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);

    }

    /**
     * Calculates the absolute field heading (in radians) from the robot's current position
     * to the center of the correct alliance goal.
     * @param alliance The current alliance, to select the correct goal.
     * @return The field-relative heading in radians.
     */
    public double calculateHeadingToGoal(GameState.Alliance alliance)
    {
        Pose robotPose = follower.getPose();

        // Select the correct goal from our presets.
        Pose targetGoal = (alliance == GameState.Alliance.BLUE)
                ? FieldPosePresets.BLUE_GOAL_TARGET
                : FieldPosePresets.RED_GOAL_TARGET;

        // Use Math.atan2 to calculate the angle between the robot's position and the goal's position.
        return Math.atan2(
                targetGoal.getY() - robotPose.getY(),
                targetGoal.getX() - robotPose.getX()
        );
    }

    public void stop() {
        turretMotor.setVelocity(0.0);
        leftFlywheel.setVelocity(0.0);
    }




}
