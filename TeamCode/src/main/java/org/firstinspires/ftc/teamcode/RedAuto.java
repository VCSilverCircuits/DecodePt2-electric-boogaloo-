package org.firstinspires.ftc.teamcode;

import com.pedropathing.Drivetrain;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.DualMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Shoot")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private DcMotorEx turret;
    private static final double TICKS_PER_REV = 537.6;

    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    private final Pose startPose = new Pose(122.0187, 123.8131, Math.toRadians(37));
    private final Pose endPose = new Pose(95.1028, 93.9813, Math.toRadians(37));
    private final Pose turnToIntake = new Pose(90.39252336448597,89.27101962616824,Math.toRadians(0));
    private final Pose intake1 = new Pose(125, 89.27101962616824, Math.toRadians(0));

    private DcMotorEx leftFlywheel, rightFlywheel;
    private DualMotor flywheel;

    private Servo servo1, servo2, servo3;
    private DcMotorEx intake;

    public static final double MAX_FLYWHEEL_RPM = 4800; // example max


    private int pathState = 0;
    private Paths paths;
    private boolean intake1Done = false;
    private boolean shooting1Start = false;

    @Override
    public void init() {
        // --- Flywheel ---
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        // --- follower ---
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();


        // --- turret ---
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        // --- servos ---
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");
        servo1.setPosition(0.45);
        servo2.setPosition(1);
        servo3.setPosition(0);

        // --- intake ---
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // --- timer ---
        pathTimer = new Timer();

        // --- paths ---
        paths = new Paths(follower);
    }


    @Override
    public void loop() {
        follower.update();
        // --- turret control ---
        Pose robotPose = follower.getPose();
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;


        // --- path updates and shooting ---
        pathState = paths.autonomousPathUpdate(pathState, robotPose);

        // --- telemetry ---
        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Turret Angle", currentAngleDeg);
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    public class Paths {

        private PathChain path1, path2, path3, path4;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;

            // --- First path to shooting position ---
            path1 = follower
                .pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

            // --- Path to intake position ---

            path2 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        endPose,
                        turnToIntake
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                .build();
            path3 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        turnToIntake,
                        intake1
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            path4 = follower
                .pathBuilder()
                .addPath(
                    new BezierLine(
                        intake1,
                        endPose
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(37))
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:
                    follow.followPath(path1);
                    setFlywheelPercent(0.6);
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    if (follow.atPose(endPose, 2, 2)) {
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
                    break;

                case 2:
                    // Activate shooter servo to launch
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1){
                        servo1.setPosition(0);

                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 1){
                        servo1.setPosition(0.45);
                        pathTimer.resetTimer();
                        pathState = 3;
                    }
                    break;

                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1) {
                        servo2.setPosition(0);
                        }
                        if (pathTimer.getElapsedTimeSeconds() >= 1) {
                            servo2.setPosition(1);
                            pathTimer.resetTimer();
                            pathState = 4;

                    }
                    break;

                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1){
                        servo3.setPosition(0.76);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 1){
                        servo3.setPosition(0);
                        pathTimer.resetTimer();

                        if (!shooting1Start) {
                            pathState = 5; // go intake
                        } else {
                            pathState = 9; // loop shooting
                        }

                        intake.setPower(-1);
                    }
                    break;

                case 5:
                    follow.followPath(path2);
                    pathState = 6;
                    break;

                case 6:
                    if (follow.atPose(turnToIntake, 2, 2)) {
                        follow.followPath(path3);
                        pathState = 7;
                    }
                    break;

                case 7:
                    if (follow.atPose(intake1, 2, 2)) {
                        intake1Done = true;
                        follow.followPath(path4); // RETURN PATH
                        pathState = 8;
                    }
                    break;

                case 8:
                    if (follow.atPose(endPose, 2, 2)) {
                        shooting1Start = true;
                        intake.setPower(0);
                        pathTimer.resetTimer();
                        pathState = 2; // ✅ correct loop
                    }
                    break;


            }
            return pathState;
        }
        private void setFlywheelRPM(double rpm) {
            double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
            leftFlywheel.setVelocity(ticksPerSecond);
            rightFlywheel.setVelocity(ticksPerSecond);
        }
        private void setFlywheelPercent(double percent) {
            percent = Math.max(0, Math.min(1, percent)); // clamp
            setFlywheelRPM(MAX_FLYWHEEL_RPM * percent);
        }

        private void stopFlywheel() {
            leftFlywheel.setVelocity(0);
            rightFlywheel.setVelocity(0);
        }
    }
}
