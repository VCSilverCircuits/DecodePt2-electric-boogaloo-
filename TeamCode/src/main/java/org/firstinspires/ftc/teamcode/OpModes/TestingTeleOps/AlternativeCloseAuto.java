package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModes.Autos.FarAutoRed;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagControllers.AprilTagTurretControllerBlue;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAimBlue;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "intake 2 blue auto")
public class AlternativeCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer autoTimer;


    OdoAimBlue turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    private AutoFlywheelConstants flywheel;
    private DcMotorEx intake;

    private AprilTagTurretControllerBlue turretController;

    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 53;


    private int timesShot = 0;


    private Paths paths;
    private int pathState = 0;


    // POSES


    private static final Pose startPose =
        new Pose(21.458, 123.738,
            Math.toRadians((37)));

    private static final Pose shootingPose =
        new Pose(95.1028, 93.9813,
            Math.toRadians((37)));

    private final Pose intakePose =
        new Pose(18.028, 59.290,
            Math.toRadians(180));
    private final Pose initialRelease =
        new Pose(15.953, 68.850,
            Math.toRadians(180));
    private final Pose repeatRelease =
        new Pose(11.505, 60.981,
            Math.toRadians(141));
    private final Pose endPose =
        new Pose(110, 93.9813,
            Math.toRadians((37)));


    @Override
    public void init() {
        turret = new OdoAimBlue(hardwareMap, follower, false);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");

        turretController = new AprilTagTurretControllerBlue(hardwareMap);
        turretController.resetController();

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        autoTimer = new Timer();

        paths = new Paths(follower);
    }

    // =========================
    // START
    // =========================

    @Override
    public void start() {
        autoTimer.resetTimer();

    }

    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.getPose();

        turret.odoAim();
        turret.update();

        servos.loop();

        if (autoTimer.getElapsedTimeSeconds() > 25) {
            pathState = 20;
        }


        PoseStorage.currentPose = robotPose;
        PoseStorage.turretRadians = turret.getTurretPosition();
        pathState = autonomousPathUpdate(pathState, robotPose);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shots", timesShot);
        telemetry.addData("Pose", robotPose);
        telemetry.update();
    }

    // =========================
    // PATHS
    // =========================

    public class Paths {
        public PathChain startToShoot;
        public PathChain shootToIntake;
        public PathChain intakeToRelease;
        public PathChain releaseToShoot;
        public PathChain shootToRelease;
        public PathChain pickupAndReleaseToShoot;
        public PathChain endPath;
        Follower follow;

        public Paths(Follower follower) {

            this.follow = follower;
            startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

            shootToIntake = follower.pathBuilder().addPath(
                    new BezierCurve(shootingPose, new Pose(63.696, 54.505), intakePose)
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

            intakeToRelease = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(18.028, 59.290),

                        new Pose(15.953, 68.850)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

            releaseToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(15.953, 68.850),

                        new Pose(46.458, 98.495)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

            shootToRelease = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(46.458, 98.495),
                        new Pose(28.645, 47.972),
                        new Pose(11.505, 60.981)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(141))

                .build();

            pickupAndReleaseToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(11.505, 60.981),

                        new Pose(46.449, 99.196)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(180))

                .build();
            endPath = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(46.449, 99.196),
                        new Pose(110, 93.9813)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(141))
                .build();
        }
    }


    public int autonomousPathUpdate(int pathState, Pose robotPose) {

        switch (pathState) {

            case 0:
                flywheel.setConstantRPM(2750);
                intake.setPower(-1);

                follower.followPath(paths.startToShoot);
                pathTimer.resetTimer();

                return 1;

            case 1:
                if (follower.atPose(shootingPose, 3, 3)) {
                    servos.StartNonSort();
                    pathTimer.resetTimer();
                    return 2;
                }
                break;

            case 2:
                if (!servos.isRunning()) {
                    follower.followPath(paths.shootToIntake);
                    pathTimer.resetTimer();
                    return 3;
                }
                break;

            case 3:
                if (follower.atPose(intakePose, 2, 2)) {
                    follower.followPath(paths.intakeToRelease);
                    pathTimer.resetTimer();
                    return 4;
                }
                break;

            case 4:
                if (follower.atPose(initialRelease, 2, 2)) {
                    follower.followPath(paths.releaseToShoot);
                    pathTimer.resetTimer();
                    return 10;
                }
                break;

            // ======================
            // LOOPING SECTION
            // ======================

            case 10:
                if (follower.atPose(shootingPose, 2, 2)) {
                    servos.StartNonSort();
                    pathTimer.resetTimer();
                    return 11;
                }
                break;

            case 11:
                if (!servos.isRunning()) {
                    follower.followPath(paths.shootToRelease);
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    return 12;
                }
                break;

            case 12:
                if (follower.atPose(repeatRelease, 2, 2)) {
                    follower.followPath(paths.pickupAndReleaseToShoot);
                    pathTimer.resetTimer();
                    return 13;
                }
                break;

            case 13:
                if (follower.atPose(shootingPose, 3, 3)) {
                    servos.StartNonSort();
                    pathTimer.resetTimer();
                    return 14;
                }
                break;

            case 14:
                if (!servos.isRunning()) {
                    return 11; // LOOP BACK (this is your repeat cycle)
                }
                break;

            // ======================
            // END
            // ======================

            case 20:
                follower.followPath(paths.endPath);
                return 21;

            case 21:
                requestOpModeStop();
                break;
        }

        return pathState;
    }
}



