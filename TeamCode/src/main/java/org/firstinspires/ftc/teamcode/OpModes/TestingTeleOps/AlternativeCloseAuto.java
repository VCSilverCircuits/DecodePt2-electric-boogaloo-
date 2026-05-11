package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.AutoFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Alternative Close Auto")
public class AlternativeCloseAuto extends OpMode {
    private DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer;
    private Timer leaveTimer;
    private double flipHeading(double heading) {
        double headingDistance = heading - 90;
        return (90 - headingDistance);
    }

    private double flipX(double x) {
        double fieldSizeX = 144;
        //double distance = x-(fieldSizeX/2);
        return (fieldSizeX - x);
    }

    double endPoseY = 93.9813;
    double endPoseX = flipX(95.1028);
    private final Pose startPose = new Pose(flipX(122.0187), 123.8131, Math.toRadians(flipHeading(37)));
    private final Pose endPose = new Pose(endPoseX, endPoseY, Math.toRadians(flipHeading(37)));
    private final Pose turnToIntake = new Pose(flipX(90.39252336448597), 89.27101962616824, Math.toRadians(flipHeading(0)));
    private final Pose intake1 = new Pose(flipX(124), 92, Math.toRadians(flipHeading(-8)));
    private final Pose releaseBalls = new Pose(flipX(125), 81, Math.toRadians(flipHeading(0)));
    private final Pose intake2Lineup = new Pose(flipX(95.15887850467287), 65, Math.toRadians(flipHeading(-3)));
    private final Pose intake2 = new Pose(flipX(131), 65, Math.toRadians(flipHeading(-7)));
    private final Pose intake3Lineup = new Pose(flipX(94.75700934579439), 42, Math.toRadians(flipHeading(-10)));
    private final Pose intake3 = new Pose(flipX(130), 42, Math.toRadians(flipHeading(-3)));
    private OdoAim turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    private AutoFlywheelConstants flywheel;


    private int pathState = 0;
    @Override
    public void init() {
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);


        turret = new OdoAim(hardwareMap, follower, false);
        flywheel = new AutoFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper", "stopper");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();


    }

    @Override
    public void loop() {
        follower.update();

        Pose robotPose = follower.getPose();
        turret.odoAim();

        flywheel.update(-follower.getVelocity().getXComponent() * 50);
        flywheel.setConstantRPM(4250);
        servos.loop();
        turret.update();
        PoseStorage.currentPose = follower.getPose();
        PoseStorage.turretRadians = turret.getTurretPosition();
    }
    public class Paths{
        private PathChain startToEnd, endToTurnToIntake, turnToIntakeToIntake1, intake1ToReleaseBalls, releaseBallsToEndPose, endPoseToLineUp, lineupToIntake2, intake2ToEndPose, endPoseToLineup3, lineup3ToIntake3, intake3ToEndPose;
        private Follower follow;
        public Paths(Follower follower) {
            this.follow = follower;

            startToEnd = follower.pathBuilder().addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading()).build();

            endToTurnToIntake = follower.pathBuilder().addPath(new BezierLine(endPose, turnToIntake))
                .setLinearHeadingInterpolation(endPose.getHeading(), intake1.getHeading()).build();

            turnToIntakeToIntake1 = follower.pathBuilder().addPath(new BezierLine(turnToIntake, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(0))).build();

            intake1ToReleaseBalls = follower.pathBuilder().addPath(new BezierCurve(intake1, new Pose(flipX(110), 80.23831775700934), releaseBalls))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(0))).build();

            releaseBallsToEndPose = follower.pathBuilder().addPath(new BezierLine(releaseBalls, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(37))).build();
            endPoseToLineUp = follower.pathBuilder().addPath(new BezierLine(endPose,intake2Lineup))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(37)),Math.toRadians(flipHeading(-3))).build();

            lineupToIntake2 = follower.pathBuilder().addPath(new BezierLine(intake2Lineup, intake2))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(-10))).build();
            intake2ToEndPose = follower.pathBuilder().addPath(new BezierCurve(intake2, new Pose(80.01869158878506,57.25233644859814),endPose))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(37))).build();
            endPoseToLineup3 = follower.pathBuilder().addPath(new BezierLine(endPose, intake3Lineup))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(-7))).build();
            lineup3ToIntake3 = follower.pathBuilder().addPath(new BezierLine(intake3Lineup, intake3))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)),Math.toRadians(flipHeading(0))).build();
            intake3ToEndPose = follower.pathBuilder().addPath(new BezierLine(intake3, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(flipHeading(0)), Math.toRadians(flipHeading(37))).build();
        }
        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {
                case 0:

            }
        }
    }
}
