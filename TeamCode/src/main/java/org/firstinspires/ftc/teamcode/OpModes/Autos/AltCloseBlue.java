package org.firstinspires.ftc.teamcode.OpModes.Autos;

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
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AltBlueFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAimBlue;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Alt Close Blue", preselectTeleOp = "Blue Tele")
public class AltCloseBlue extends OpMode {

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
    boolean twoShotsDone = false;

    // Pathing
    private Paths paths;
    private int pathState = 0;

    // Poses
    private static final Pose startPose = new Pose(21.5, 123.7, Math.toRadians((141)));
    private static final Pose firingPose = new Pose(55.4, 91.15, Math.toRadians((141)));

    private static final Pose intakePose = new Pose(12.5, 67.1, Math.toRadians(180));
    private static final Pose initialRelease = new Pose(16.0, 76.9, Math.toRadians(180));
    private final Pose repeatRelease = new Pose(14.0, 65.3, Math.toRadians(156));
    private static final Pose closeIntake = new Pose(20.2,97.3, Math.toRadians(180));
    private static final Pose backOffFromRamp = new Pose(11.7,55.5, Math.toRadians((156)));

    private static boolean hasStartedFlywheel = false;
    private boolean stateMachineIterator = false;

    @Override
    public void init() {

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        turret = new OdoAimBlue(hardwareMap, follower, false, true);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        poseTimer = new Timer();

        paths = new Paths(follower);

        //This offset applies through the entire opMode.
        //turret.manualOffsetRad = -Math.toRadians(8);

        //This ensures that the flywheel does not reactivate at the end after turning it off
        hasStartedFlywheel = false;
    }


    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.getPose();


        turret.odoAim();
        flywheel.update(-follower.getVelocity().getXComponent() * 50);

        if (!hasStartedFlywheel) {
            flywheel.setConstantRPM(3000);
            flywheel.setConstantHood(70);
            hasStartedFlywheel = true;
        }

        if (!endTriggered && poseTimer.getElapsedTimeSeconds() >= 28.5) {
            endTriggered = true;
            follower.followPath(paths.endPath);
            //Undo manual turret offset before teleop starts
            turret.manualOffsetRad = 0;
            turret.idle();
        }
        if (timesShot >= 2 && !twoShotsDone){
            twoShotsDone = true;
            pathState = 12;
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
        telemetry.addData("robotX,", robotPose.getX());
        telemetry.addData("robotY",robotPose.getY());
        telemetry.update();

        // End condition
        PoseStorage.currentPose = new Pose(follower.getPose().getX()+35, follower.getPose().getY()-14, follower.getHeading());
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
        public PathChain firingToCloseIntake;
        public PathChain closeIntakeToFiring;
        public PathChain repeatReleaseToShoot;
        public PathChain backOffPointToShoot;
        public PathChain intakeToFiring;
        public PathChain backOffLever;

        private Follower follow;

        public Paths(Follower follower) {

            this.follow = follower;

            startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firingPose.getHeading())

                .build();

            shootToIntake = follower.pathBuilder().addPath(
                    new BezierCurve(
                            firingPose,
                            new Pose(54.6, 67.5),
                            intakePose
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

            intakeToRelease = follower.pathBuilder().addPath(
                    new BezierCurve(
                        intakePose,
                        new Pose(28.5,66.3),
                        initialRelease
                    )
                ).setLinearHeadingInterpolation(
                        intakePose.getHeading(),
                            initialRelease.getHeading()
                    )
                .build();

            releaseToShoot = follower.pathBuilder().addPath(
                    new BezierLine(
                        initialRelease,
                        firingPose
                    )
                ).setLinearHeadingInterpolation(initialRelease.getHeading(), firingPose.getHeading())

                .build();
            repeatReleaseToShoot = follower.pathBuilder().addPath(
                new BezierCurve(
                    repeatRelease,
                    new Pose(57.2, 72.3),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                    repeatRelease.getHeading(),
                            firingPose.getHeading()
                    )
                .build();

            backOffPointToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    backOffFromRamp,
                                    new Pose(57.2, 65.3),
                                    firingPose
                            )
                    ).setLinearHeadingInterpolation(
                            repeatRelease.getHeading(),
                            firingPose.getHeading()
                    )
                    .build();

            shootToRelease = follower.pathBuilder().addPath(
                    new BezierCurve(
                        firingPose,
                        new Pose(57.2, 72.3),
                        repeatRelease
                    )
                ).setLinearHeadingInterpolation(
                        firingPose.getHeading(),
                            repeatRelease.getHeading()
                    )

                .build();

            pickupAndReleaseToShoot = follower.pathBuilder().addPath(
                    new BezierCurve(
                        repeatRelease,
                        new Pose(49.5, 53.3),
                        firingPose
                    )
                ).setLinearHeadingInterpolation(
                        repeatRelease.getHeading(),
                            firingPose.getHeading()
                    )
                .build();

            endPath = follower.pathBuilder().addPath(
                    new BezierLine(
                        //new Pose(46.449, 99.196),
                        firingPose,
                            new Pose(55.4, 111, Math.toRadians((141))) // Off the line end position
                    )
                ).setLinearHeadingInterpolation(
                        Math.toRadians(141),
                            Math.toRadians(141)
                    )
                .build();

            firingToCloseIntake = follower.pathBuilder().addPath(
                new BezierLine(
                    firingPose,
                    closeIntake
                )
            ).setLinearHeadingInterpolation(
                    Math.toRadians(180),
                            Math.toRadians(180)
                    )
                .build();
            closeIntakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    closeIntake,
                    new Pose(42.2, 82.9),
                    firingPose
                )
            ) .setLinearHeadingInterpolation(
                    Math.toRadians(180),
                            Math.toRadians(180)
                    )
                .build();
            intakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    intakePose,
                    new Pose(61.0, 59.7),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                    intakePose.getHeading(),
                            firingPose.getHeading()
                    )
                .build();
            backOffLever = follower.pathBuilder().addPath(
                    new BezierLine(
                            repeatRelease,
                            backOffFromRamp
                    )
            ).setLinearHeadingInterpolation(
                    repeatRelease.getHeading(),
                            repeatRelease.getHeading()
                    )
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
                    if (pathTimer.getElapsedTimeSeconds() > 2 || follow.atPose(firingPose, 2, 2)) {
                        servos.StartNonSort();
                        pathState = 3;
                    }
                    break;

                case 3:
                    pathTimer.resetTimer();
                    if (!servos.isRunning()) {
                        intake.setPower(-1);
                        follow.followPath(paths.shootToIntake);
                        pathTimer.resetTimer();
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (follow.atPose(intakePose,2,2) || pathTimer.getElapsedTimeSeconds() > 2 ) {
                        follow.followPath(paths.intakeToFiring);
                        pathState = 5;
                        pathTimer.resetTimer();
                    }
                    break;

                case 5:
                    if (follow.atPose(firingPose, 2, 2)) {
                        servos.StartNonSort();
                        pathState = 7;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2){
                        intake.setPower(1);
                    }
                    break;

                case 7:
                    if (!servos.isRunning()){
                        follow.followPath(shootToRelease);
                        pathTimer.resetTimer();
                        pathState = 8;
                        intake.setPower(-1);
                    }
                    break;
                case 8:
                    if (follow.atPose(repeatRelease, 2, 2)) {
                        if (!stateMachineIterator) {
                            stateMachineIterator = true;
                            pathTimer.resetTimer();
                        }
                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                            follow.followPath(backOffLever);
                            pathState = 9;
                            pathTimer.resetTimer();
                        }
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 3) {
                        follow.followPath(backOffLever);
                        pathState = 9;
                        pathTimer.resetTimer();
                    }
                    break;
                case 9:
                    /*if (follow.atPose(backOffFromRamp, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() >= 3.4){
                            follow.followPath(backOffPointToShoot);
                            pathState = 10;
                        }
                        } else */if (pathTimer.getElapsedTimeSeconds() >= 1.3) {
                        follow.followPath(backOffPointToShoot);
                        pathState = 10;
                        pathTimer.resetTimer();
                    }
                    break;
                case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5){
                        intake.setPower(1);
                    }
                    if (follow.atPose(firingPose, 2, 2)) {
                        pathTimer.resetTimer();
                        servos.StartNonSort();
                        pathState = 11;
                    }
                    break;
                case 11:
                    if (!servos.isRunning()) {
                        timesShot = timesShot+1;
                        follow.followPath(shootToRelease);
                        intake.setPower(-1);
                        pathState = 7;
                    }
                    break;
                case 12:
                    follow.followPath(firingToCloseIntake);
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    pathState = 13;
                    break;
                case 13:
                    if (follow.atPose(closeIntake,2,2)){
                        follow.followPath(closeIntakeToFiring);
                        pathState = 14;
                        pathTimer.resetTimer();
                    }
                    break;
                case 14:
                    //backspin once we've backed away
                    if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                        intake.setPower(1);
                    }
                    if (follow.atPose(firingPose,2,2)){
                        servos.StartNonSort();
                        pathState = 15;
                    }
                    break;
                case 15:
                    if (!servos.isRunning()){
                        follow.followPath(endPath);
                        pathState = 16;
                    }
                    break;
                case 16:
                    if (follow.atPose(firingPose,2,2)){
                        intake.setPower(0);
                        flywheel.disable();
                    }
                    }
                return pathState;
            }
        }
    }



