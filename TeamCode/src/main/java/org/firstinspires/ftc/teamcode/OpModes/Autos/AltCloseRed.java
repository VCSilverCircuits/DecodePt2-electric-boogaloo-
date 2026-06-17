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
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Alt Close Red")
public class AltCloseRed extends OpMode {

    // Hardware
    private DcMotorEx intake;
    double timesShot = 0;

    private Follower follower;
    private MecanumConstants mecanumConstants;
    private OdoAim turret;
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
    private static final Pose startPose = new Pose(121.6, 121.6, Math.toRadians(39));
    private static final Pose firingPose = new Pose(88.598, 88.6, Math.toRadians(39));

    private static final Pose intakePose = new Pose(131.6, 65.1, Math.toRadians(0));
    private static final Pose initialRelease = new Pose(128.0, 76.9, Math.toRadians(0));
    private final Pose repeatRelease = new Pose(130, 61.5, Math.toRadians(26));
    private static final Pose closeIntake = new Pose(123.8, 95, Math.toRadians(0));
    private static final Pose backOffFromRamp = new Pose(131,54.2, Math.toRadians((31)));

    private static boolean hasStartedFlywheel = false;
    private boolean StateMachineUtilityBooleanVariableJustToMakeFutureEditorsRageAboutTheLackOfDocumentation = false;

    @Override
    public void init() {

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        turret = new OdoAim(hardwareMap, follower, false);
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
            flywheel.setConstantRPM(3100);
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
        PoseStorage.currentPose = new Pose(follower.getPose().getX()-23, follower.getPose().getY()-7, follower.getHeading());
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
                        new Pose(89.4, 65),
                        intakePose
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            intakeToRelease = follower.pathBuilder().addPath(
                new BezierCurve(
                    intakePose,
                    new Pose(115.5, 66.3),
                    initialRelease
                )
            ).setLinearHeadingInterpolation(
                intakePose.getHeading(),
                initialRelease.getHeading()
            ).build();

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
                    new Pose(86.8, 72.3),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                repeatRelease.getHeading(),
                firingPose.getHeading()
            ).build();

            backOffPointToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    backOffFromRamp,
                                    new Pose(86.8, 65.3),
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
                    new Pose(86.8, 72.3),
                    repeatRelease
                )
            ).setLinearHeadingInterpolation(
                firingPose.getHeading(),
                repeatRelease.getHeading()
            ).build();

            pickupAndReleaseToShoot = follower.pathBuilder().addPath(
                new BezierCurve(
                    repeatRelease,
                    new Pose(94.5, 53.3),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                repeatRelease.getHeading(),
                firingPose.getHeading()
            ).build();

            endPath = follower.pathBuilder().addPath(
                new BezierLine(
                    //new Pose(97.551, 99.196),
                    firingPose,
                        new Pose(88.6, 111, Math.toRadians(39))
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(39),
                Math.toRadians(39)
            )
                    .build();

            firingToCloseIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                    firingPose,
                    new Pose(95,94),
                    closeIntake
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(0)
            )
                    .build();
            closeIntakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    closeIntake,
                    new Pose(101.8, 82.9),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(0)
            )
                    .build();
            intakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    intakePose,
                    new Pose(83.0, 59.7),
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
                    if (!servos.isRunning()) {
                        intake.setPower(-1);
                        follow.followPath(paths.shootToIntake);
                        pathTimer.resetTimer();
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (follow.atPose(intakePose,2,2) || pathTimer.getElapsedTimeSeconds() > 4 ) {
                        follow.followPath(paths.intakeToFiring);
                        pathTimer.resetTimer();
                        pathState = 5;
                    }
                    break;

                case 5:
                    if (follow.atPose(firingPose, 2, 2)) {
                        servos.StartNonSort();
                        pathState = 7;
                        pathTimer.resetTimer();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
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
                        //reset timer once when we get to the gate, so we can track how long we want to stay holding it open
                        if(!StateMachineUtilityBooleanVariableJustToMakeFutureEditorsRageAboutTheLackOfDocumentation) {
                            pathTimer.resetTimer();
                            StateMachineUtilityBooleanVariableJustToMakeFutureEditorsRageAboutTheLackOfDocumentation = true;
                        }
                        if (pathTimer.getElapsedTimeSeconds() >= 0.05){
                            follow.followPath(backOffLever);
                            pathState = 9;
                            pathTimer.resetTimer();
                        }
                    } else if (pathTimer.getElapsedTimeSeconds() >= 3.25) {
                        follow.followPath(backOffLever);
                        pathState = 9;
                        pathTimer.resetTimer();
                    }
                    break;
                case 9:
                    /*if (follow.atPose(backOffFromRamp,2,2)) {
                        if (pathTimer.getElapsedTimeSeconds() >= 3.4){
                            follow.followPath(backOffPointToShoot);
                            pathState = 10;
                        }
                    } else*/
                    if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
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



