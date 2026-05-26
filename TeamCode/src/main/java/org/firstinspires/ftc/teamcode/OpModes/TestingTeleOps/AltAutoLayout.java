package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAimBlue;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Alt Auto Layout Testing")
public class AltAutoLayout extends OpMode {

    // Hardware
    private DcMotorEx intake;
    double timesShot = 0;

    private Follower follower;
    private MecanumConstants mecanumConstants;
    private OdoAimBlue turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    private AutoFlywheelConstants flywheel;
    private Timer pathTimer;
    private Timer poseTimer;

    boolean intakeDelayStarted = false;
    boolean endTriggered = false;
    boolean isAtShootingPose = false;

    // Pathing
    private Paths paths;
    private int pathState = 0;

    // Poses
    private static final Pose startPose = new Pose(21.458, 123.738, Math.toRadians((141)));
    private static final Pose firingPose = new Pose(55.402, 88.15, Math.toRadians((180)));

    private static final Pose intakePose = new Pose(18.028, 59.290, Math.toRadians(180));
    private static final Pose initialRelease = new Pose(15.953, 72.850, Math.toRadians(180));
    private final Pose repeatRelease = new Pose(11.505, 61.981, Math.toRadians(145));

    @Override
    public void init() {

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        turret = new OdoAimBlue(hardwareMap, follower, false);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        poseTimer = new Timer();

        paths = new Paths(follower);
    }

    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.getPose();


        turret.odoAim();

        flywheel.update(-follower.getVelocity().getXComponent() * 50);
        flywheel.setConstantRPM(3000);

        if (!endTriggered && poseTimer.getElapsedTimeSeconds() >= 27.5) {
            endTriggered = true;
            follower.followPath(paths.endPath);
            turret.idle();
        }


        servos.loop();
        turret.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("times Shot", timesShot);
        telemetry.addData("pose", robotPose);
        telemetry.addData("at shooting Pose", isAtShootingPose);
        telemetry.addData("is running?", servos.isRunning());
        telemetry.addData("current flywheel rpm", flywheel.getCurrentRPM());
        telemetry.addData("target flywheel rpm", flywheel.getTargetRPM());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

        // End condition
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.turretRadians = turret.getTurretPosition();

        if (!endTriggered) {
            pathState = paths.autonomousPathUpdate(pathState, robotPose);
        }
    }

    public class Paths {

        public PathChain startToShoot;
        public PathChain shootToIntake;
        public PathChain intakeToRelease;
        public PathChain releaseToShoot;
        public PathChain shootToRelease;
        public PathChain pickupAndReleaseToShoot;
        public PathChain endPath;

        private Follower follow;

        public Paths(Follower follower) {

            this.follow = follower;

            startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firingPose.getHeading())

                .build();

            shootToIntake = follower.pathBuilder().addPath(
                    new BezierCurve(firingPose, new Pose(63.696, 54.505), intakePose)
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

            intakeToRelease = follower.pathBuilder().addPath(
                    new BezierCurve(
                        intakePose,
                        new Pose(28.45,66.335),
                        initialRelease
                    )
                ).setLinearHeadingInterpolation(intakePose.getHeading(), initialRelease.getHeading())
                .build();

            releaseToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        initialRelease,
                        firingPose
                    )
                ).setLinearHeadingInterpolation(initialRelease.getHeading(), firingPose.getHeading())

                .build();

            shootToRelease = follower.pathBuilder().addPath(
                    new BezierCurve(
                        firingPose,
                        new Pose(28.645, 47.972),
                        repeatRelease
                    )
                ).setLinearHeadingInterpolation(firingPose.getHeading(), repeatRelease.getHeading())

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
                        firingPose
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(141))
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {

            switch (pathState) {

                case 0:
                    pathTimer.resetTimer();
                    poseTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    follow.followPath(paths.startToShoot);
                    pathTimer.resetTimer();
                    pathState = 2;
                    break;

                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 3 || follow.atPose(firingPose, 2, 2)) {
                        servos.StartNonSort();
                        pathState = 3;
                    }
                    break;

                case 3:
                    pathTimer.resetTimer();
                    if (!servos.isRunning()) {
                        follower.setMaxPower(0.8);
                        intake.setPower(-1);
                        follow.followPath(paths.shootToIntake);
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        follow.followPath(paths.intakeToRelease);
                        pathState = 5;
                    }
                    break;

                case 5:
                    if (follow.atPose(initialRelease, 3, 3)) {
                        follower.setMaxPower(1);
                        follow.followPath(paths.releaseToShoot);
                        pathTimer.resetTimer();
                        pathState = 6;
                    }
                    break;

                case 6:
                    if (follow.atPose(firingPose, 2, 2)) {
                        servos.StartNonSort();
                        pathState = 7;
                    }
                    break;
                case 7:
                    if (!servos.isRunning()){
                        follow.followPath(shootToRelease);
                        pathTimer.resetTimer();
                        pathState = 8;
                    }
                    break;
                case 8:
                    if (follow.atPose(repeatRelease, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() >= 5){
                            follow.followPath(releaseToShoot);
                            pathState = 9;
                        }
                        }

                    break;

                case 9:
                    if (follow.atPose(firingPose, 2, 2)) {
                        pathTimer.resetTimer();
                        servos.StartNonSort();
                        pathState = 10;
                    }
                    break;
                case 10:
                    if (!servos.isRunning()) {
                        follow.followPath(shootToRelease);
                        pathState = 7;
                    }
                    break;
                    }
                return pathState;
            }
        }
    }



