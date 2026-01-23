package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp
public class GoalPositionTestTele extends OpMode {
    private AprilTagTurretControllerRed turretController;
    private Follower follower;
    private DcMotorEx turret;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;
    private static final double GOAL_X = 130.99065420560748;
    private static final double GOAL_Y = 135.70093457943926;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(83.44, 15.25, Math.toRadians(0))); // example start
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretController = new AprilTagTurretControllerRed(hardwareMap);


    }
    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );
        Pose robotPose = follower.getPose();
        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Vision Lock", turretController.isLocked());
        telemetry.addData("Mode", turretController.isLocked() ? "Vision" : "Odometry");
        telemetry.addData("robotX", robotPose.getX());
        telemetry.addData("robotY",robotPose.getY());
        telemetry.addData("GoalX", GOAL_X);
        telemetry.addData("GoalY", GOAL_Y);
        telemetry.update();
    }
}
