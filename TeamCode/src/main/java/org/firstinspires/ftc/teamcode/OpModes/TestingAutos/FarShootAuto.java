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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous( name = "Far Auto Red")
public class FarShootAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer;
    private Timer leaveTimer;
    MecanumConstants mecanumConstants;

    private OdoAim turret;
    public static final double TICKS_PER_REV = 28;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 53;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    ColorSensors sensors;
    private ServoGroup servos;

    int timesShot = 0;

    private static final Pose startPose = new Pose(84.03738317757008, 10.018691588785039, Math.toRadians(0));
    private static final Pose intake1 = new Pose(128.89719626168227, 36, Math.toRadians(0));
    private DcMotorEx leftFlywheel, rightFlywheel;
    private DcMotorEx intake;
    public static final double MAX_FLYWHEEL_RPM = 6000;
    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        mecanumConstants = new MecanumConstants(); // assign to the class-level field

        turret = new OdoAim(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(
            hardwareMap,
            "frontFlipper", "backFlipper", "leftFlipper"
        );
        pathTimer = new Timer();
        leaveTimer = new Timer();

        // Paths
        paths = new Paths(follower);

    }

    @Override
    public void loop() {
        follower.update();
        Pose robotPose = follower.getPose();
        servos.loop();
        turret.update();
        turret.odoAim();

        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);
        follower.update();

        pathState = paths.autonomousPathUpdate(pathState, robotPose);

    }

    public static class Paths {
        public PathChain Path1, Path2;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;
            Path1 = follower.pathBuilder().addPath(new BezierCurve(startPose,new Pose(75.636, 35.991), intake1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intake1.getHeading()).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(128.897, 36.000),
                        new Pose(75.636, 35.991),
                        new Pose(84.037, 10.019)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:

            }

            return pathState;
        }
    }
}
