package org.firstinspires.ftc.teamcode.OpModes.TestingAutos;

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
    private static final Pose startPose = new Pose(122.542, 123.738, Math.toRadians(39));
    private static final Pose firingPose = new Pose(88.598, 88.15, Math.toRadians(0));

    private static final Pose intakePose = new Pose(131.56078504672897, 64.12177570093458, Math.toRadians(0));
    private static final Pose initialRelease = new Pose(128.047, 76.850, Math.toRadians(0));
    private final Pose repeatRelease = new Pose(132.1, 60.3, Math.toRadians(30));
    private static final Pose closeIntake = new Pose(123.82158878504673, 90.8, Math.toRadians(0));

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
    }

    @Override
    public void loop() {

        follower.update();

        Pose robotPose = follower.getPose();


        turret.odoAim();
        flywheel.update(-follower.getVelocity().getXComponent() * 50);

        flywheel.setConstantRPM(3000);
        flywheel.setConstantHood(70);

        if (!endTriggered && poseTimer.getElapsedTimeSeconds() >= 28.5) {
            endTriggered = true;
            follower.followPath(paths.endPath);
            turret.idle();
        }
        if (timesShot >= 2 && !twoShotsDone){
            twoShotsDone = true;
            pathState = 11;
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
        public PathChain firingToCloseIntake;
        public PathChain closeIntakeToFiring;
        public PathChain repeatReleaseToShoot;
        public PathChain intakeToFiring;

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
                        new Pose(89.3787663551402, 61.505),
                        intakePose
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            intakeToRelease = follower.pathBuilder().addPath(
                new BezierCurve(
                    intakePose,
                    new Pose(115.55, 66.335),
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
                    new Pose(86.80411975657465, 72.26046446424694),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                repeatRelease.getHeading(),
                firingPose.getHeading()
            ).build();

            shootToRelease = follower.pathBuilder().addPath(
                new BezierCurve(
                    firingPose,
                    new Pose(86.80411975657465, 72.26046446424694),
                    repeatRelease
                )
            ).setLinearHeadingInterpolation(
                firingPose.getHeading(),
                repeatRelease.getHeading()
            ).build();

            pickupAndReleaseToShoot = follower.pathBuilder().addPath(
                new BezierCurve(
                    repeatRelease,
                    new Pose(94.53725929145838, 53.33895283633994),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                repeatRelease.getHeading(),
                firingPose.getHeading()
            ).build();
            endPath = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(97.551, 99.196),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(39),
                Math.toRadians(39)
            ).build();
            firingToCloseIntake = follower.pathBuilder().addPath(
                new BezierLine(
                    firingPose,
                    closeIntake
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(0)
            ).build();
            closeIntakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    closeIntake,
                    new Pose(101.79432710280373, 82.9161214953271),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                Math.toRadians(0),
                Math.toRadians(0)
            ).build();
            intakeToFiring = follower.pathBuilder().addPath(
                new BezierCurve(
                    intakePose,
                    new Pose(82.99065420560748, 59.663551401869164),
                    firingPose
                )
            ).setLinearHeadingInterpolation(
                intakePose.getHeading(),
                firingPose.getHeading()
            ).build();
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
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (follow.atPose(intakePose,2,2) || pathTimer.getElapsedTimeSeconds() > 4 ) {
                        follow.followPath(paths.intakeToFiring);
                        pathState = 5;
                    }
                    break;

                case 5:
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
                        if (pathTimer.getElapsedTimeSeconds() >= 3.4){
                            follow.followPath(repeatReleaseToShoot);
                            pathState = 9;
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 3.4){
                            intake.setPower(1);
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
                        timesShot = timesShot+1;
                        follow.followPath(shootToRelease);
                        intake.setPower(-1);
                        pathState = 7;
                    }
                    break;
                case 11:
                    follow.followPath(firingToCloseIntake);
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    pathState = 12;
                    break;
                case 12:
                    if (follow.atPose(closeIntake,2,2)){
                        follow.followPath(closeIntakeToFiring);
                        pathState = 13;
                    }
                    break;
                case 13:
                    if (follow.atPose(firingPose,2,2)){
                        servos.StartNonSort();
                        pathState = 14;
                    }
                    break;
                case 14:
                    if (!servos.isRunning()){
                        follow.followPath(endPath);
                        pathState = 15;
                    }
                    break;
                case 15:
                    if (follow.atPose(firingPose,2,2)){
                        intake.setPower(0);
                        flywheel.disable();
                    }
            }
            return pathState;
        }
    }
}



