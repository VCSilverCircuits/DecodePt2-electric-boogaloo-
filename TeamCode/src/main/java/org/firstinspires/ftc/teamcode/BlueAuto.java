package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto")
public class BlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private DcMotorEx turret;
    private AprilTagTurretControllerRed turretController;

    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    private final Pose startPose = new Pose(21.30841121495327, 122.69158878504673, Math.toRadians(141));
    private final Pose endPose = new Pose(45.08411214953271, 99.14018691588785, Math.toRadians(141));
    private final Pose intake1 = new Pose(14.355140186915888, 83.66355140186916, Math.toRadians(180));

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        // --- follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        // --- turret ---
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new AprilTagTurretControllerRed(hardwareMap);

        // --- timers ---
        pathTimer = new Timer();

        // --- paths ---
        paths = new Paths(follower);
    }

    @Override
    public void loop() {

        follower.update();

        // --- turret control ---
        Pose robotPose = follower.getPose();
        double currentTurretAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;

        // --- update path ---
        pathState = paths.autonomousPathUpdate(pathState, robotPose);

        // --- telemetry ---
        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Turret Angle", currentTurretAngleDeg);
        telemetry.addData("Vision Lock", turretController.isLocked());
        telemetry.addData("Mode", turretController.isLocked() ? "Vision" : "IMU/Odometry");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    public class Paths {

        private PathChain path1, path2;
        private Follower follow;
        private static final double POSITION_TOLERANCE = 1.5;
        private final double HEADING_TOLERANCE = Math.toRadians(2);

        public Paths(Follower follower) {
            this.follow = follower;

            path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

            path2  = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(45.084, 99.140),
                        new Pose(63.028, 82.991),
                        intake1
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(intake1.getHeading()))
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:
                    follow.followPath(path1);
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    if (follow.atPose(endPose, 0.8,0.8)){
                        pathState =2;
                    }

                    break;

                case 2:
                    follow.followPath(path2);
                    pathState = 3;
                    break;

                case 3:
                    if (follow.atPose(intake1,0.8,0.8)) {
                        pathState = 4;
                    }
                    break;

                case 4:
                    requestOpModeStop();
                    break;
            }
            return pathState;
        }
    }
}
