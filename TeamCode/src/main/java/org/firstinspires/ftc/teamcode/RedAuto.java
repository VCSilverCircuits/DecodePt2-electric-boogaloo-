package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants;

@Autonomous(name = "Red Auto Shoot")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    MecanumConstants mecanumConstants;

    private DcMotorEx turret;
    private static final double TICKS_PER_REV = 537.6;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    int timesShot = 0;

    private AprilTagTurretControllerRed turretController;



    private final Pose startPose = new Pose(122.0187, 123.8131, Math.toRadians(37));
    private final Pose endPose = new Pose(95.1028, 93.9813, Math.toRadians(37));
    private final Pose turnToIntake = new Pose(90.39252336448597, 89.27101962616824, Math.toRadians(0));
    private final Pose intake1 = new Pose(127, 93, Math.toRadians(0));
    private final Pose releaseBalls = new Pose(128, 82, Math.toRadians(0));
    private final Pose intake2 = new Pose(125,57, Math.toRadians(0));
    private DcMotorEx leftFlywheel, rightFlywheel;
    private DualMotor flywheel;
    private Servo servo1, servo2, servo3;
    private DcMotorEx intake;

    boolean shooting1Start = false;

    public static final double MAX_FLYWHEEL_RPM = 10000;

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        // Flywheel
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        mecanumConstants = new MecanumConstants(); // assign to the class-level field
        mecanumConstants.maxPower = 0.2;

        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new AprilTagTurretControllerRed(hardwareMap);
        turretController.resetController();

        // Follower
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        // Turret
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Servos
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");
        servo1.setPosition(0.45);
        servo2.setPosition(1);
        servo3.setPosition(0);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Timer
        pathTimer = new Timer();

        // Paths
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        follower.update();
        Pose robotPose = follower.getPose();

        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        double power = turretController.getTurretPower(currentAngleDeg);

        // ONLY run AprilTag tracking during shooting states
        if (pathState == 2 || pathState == 3 || pathState == 4) {
            turret.setPower(power);
        } else {
            turret.setPower(0);
        }

        pathState = paths.autonomousPathUpdate(pathState, robotPose);

        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Turret Angle", currentAngleDeg);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Path State", pathState);
        telemetry.addData("shootsFired!!", timesShot);
        telemetry.update();
    }

    @Override
    public void start(){
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        turretController.setPosition(0, currentAngleDeg);
    }

    public class Paths {

        private PathChain startToEnd, endToTurnToIntake, turnToIntakeToIntake1, intake1ToReleaseBalls, releaseBallsToEndPose, endPoseToIntake2, intake2ToEndPose;
        private Follower follow;

        public Paths(Follower follower) {
            this.follow = follower;

            startToEnd = follower.pathBuilder().addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading()).build();

            endToTurnToIntake = follower.pathBuilder().addPath(new BezierLine(endPose, turnToIntake))
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0)).build();

            turnToIntakeToIntake1 = follower.pathBuilder().addPath(new BezierLine(turnToIntake, intake1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            intake1ToReleaseBalls = follower.pathBuilder().addPath(new BezierCurve(intake1, new Pose(117.107476635514, 80.23831775700934), releaseBalls))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            releaseBallsToEndPose = follower.pathBuilder().addPath(new BezierLine(releaseBalls, endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37)).build();

            endPoseToIntake2 = follower.pathBuilder().addPath(new BezierCurve(endPose,new Pose(66.803738317757, 42.17289719626167), intake2))
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0)).build();

            intake2ToEndPose = follower.pathBuilder().addPath(new BezierCurve(intake2, new Pose(79.813,48.407), endPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37)).build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0: // Start -> Shoot preload
                    follow.followPath(startToEnd);
                    setFlywheelPercent(0.4);
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;

                case 1:
                    if (follow.atPose(endPose, 2, 2)) {
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
                    break;

                case 2: // Shoot
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1) {
                        servo1.setPosition(0);
                        //0
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 1) {
                        servo1.setPosition(0.45);
                        //0.45
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1) {
                        servo3.setPosition(0.76);
                    }
                    if (pathTimer.getElapsedTimeSeconds() >= 1) {
                        servo3.setPosition(0);
                        pathTimer.resetTimer();
                        intake.setPower(-1);

                        // increment timesShot once
                        timesShot++;

                        if (timesShot == 1) {
                            pathState = 5; // go to first intake
                        } else if (timesShot == 2) {
                            pathState = 10; // go to second intake
                        } else {
                            pathState = 15; // final state after second round
                        }
                    }
                    break;


                // INTAKE FIRST CYCLE
                case 5:
                    follow.followPath(endToTurnToIntake);
                    pathState = 6;
                    break;

                case 6:
                    if (follow.atPose(turnToIntake, 2, 2)) {
                        mecanumConstants.maxPower = 0.2;
                        follow.followPath(turnToIntakeToIntake1);
                        pathTimer.resetTimer();
                        pathState = 7;
                    }
                    break;

                case 7:
                    if (follow.atPose(intake1, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 3){
                            intake.setPower(1);
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 2){
                            follow.followPath(intake1ToReleaseBalls);
                            mecanumConstants.maxPower = 0.3;
                            pathTimer.resetTimer();
                            pathState = 8;
                        }
                    }
                    break;

                case 8:
                    if (follow.atPose(releaseBalls, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                            intake.setPower(-1);
                            follow.followPath(releaseBallsToEndPose);
                            pathTimer.resetTimer();
                            pathState = 9;
                        }
                    }
                    break;

                case 9:
                    if (follow.atPose(endPose, 2, 2)) {
                        mecanumConstants.maxPower = 0.55;
                        intake.setPower(0);
                        pathTimer.resetTimer();
                        pathState = 2; // final idle
                    }
                    break;

                case 10: // Move to second intake
                    follow.followPath(endPoseToIntake2);
                    intake.setPower(-1);
                    pathTimer.resetTimer(); // reset timer here once
                    pathState = 11;
                    break;

                case 11: // Intake 2
                    if (follow.atPose(intake2, 2, 2)) {
                        if (pathTimer.getElapsedTimeSeconds() > 3) {
                            intake.setPower(1);
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 4) {
                            intake.setPower(-1);
                            pathTimer.resetTimer(); // reset for next state
                            pathState =12;
                        }
                    }
                    break;
                case 12:
                    follow.followPath(intake2ToEndPose);
                    pathState = 13;
                   break;
                case 13:
                    if (follow.atPose(endPose,2,2)){
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
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
