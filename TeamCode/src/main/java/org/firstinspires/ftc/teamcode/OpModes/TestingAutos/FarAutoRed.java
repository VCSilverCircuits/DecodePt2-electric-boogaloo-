package org.firstinspires.ftc.teamcode.OpModes.TestingAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Far Auto Red")
public class FarAutoRed extends OpMode {

    // Hardware
    private DcMotorEx leftFlywheel, rightFlywheel, intake;
    private static final double TICKS_PER_REV = 28;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 53;
    double timesShot = 0;


    private Follower follower;
    private MecanumConstants mecanumConstants;
    private OdoAim turret;
    private ColorSensors sensors;
    private ServoGroup servos;
    FlywheelConstants flywheel;
    private Timer pathTimer;

    // Pathing
    private Paths paths;
    private int pathState = 0;

    // Poses
    private static final Pose startPose = new Pose(84.037, 10.019, Math.toRadians(0));
    private static final Pose intake1 = new Pose(128.897, 36, Math.toRadians(0));

    @Override
    public void init() {
        // Motors
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        intake = hardwareMap.get(DcMotorEx.class, "intake"); // Make sure name matches config
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Follower & drive constants
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.setStartingPose(startPose);

        mecanumConstants = new MecanumConstants();

        // Turret
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new FlywheelConstants(hardwareMap, follower);
        // Sensors & servos
        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(hardwareMap, "frontFlipper", "backFlipper", "leftFlipper");

        // Timers
        pathTimer = new Timer();

        // Paths
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        flywheel.update(-follower.getVelocity().getXComponent() * 50);
        Pose robotPose = follower.getPose();

        servos.loop();
        turret.update();
        turret.odoAim();
        flywheel.enable();
        telemetry.addData("current flywheel rpm", flywheel.getCurrentRPM());
        telemetry.addData("target flywheel rpm", flywheel.getTargetRPM());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("times Shot", timesShot);
        telemetry.update();
        // Pathing
        pathState = paths.autonomousPathUpdate(pathState, robotPose);
    }

    // Inner class for paths
    public class Paths {
        private PathChain Path1, Path2;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;

            Path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(75.636, 35.991), intake1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intake1.getHeading())
                .build();

            Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake1, new Pose(75.636, 35.991), startPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), startPose.getHeading())
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:
                    if (flywheel.atSpeed(450) && pathTimer.getElapsedTimeSeconds() > 3) {
                        servos.StartNonSort();
                        follow.setMaxPower(1);
                        intake.setPower(-1);
                        if (timesShot == 1){
                            pathState = 4;
                        } else pathState = 1;
                    }
                    break;

                case 1:
                    if (!servos.isRunning()) {
                        follow.followPath(Path1);
                        intake.setPower(1);
                        pathState = 2;
                    }
                    break;

                case 2:
                    if (follow.atPose(intake1, 2, 2)) {
                        pathTimer.resetTimer();
                        intake.setPower(-1);
                        follow.followPath(Path2);
                        pathState = 3;
                    }
                    break;

                case 3:
                    if (follow.atPose(startPose, 2, 2)) {
                        timesShot++;
                        pathState = 0;
                        pathTimer.resetTimer();  // reset here
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
