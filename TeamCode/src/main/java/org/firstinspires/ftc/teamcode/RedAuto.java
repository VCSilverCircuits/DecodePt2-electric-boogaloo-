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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Shoot")
public class RedAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    private DcMotorEx turret;
    private AprilTagTurretControllerRed turretController;
    MecanumConstants mecanumConstants;

    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    private final Pose startPose = new Pose(122.0187, 123.8131, Math.toRadians(37));
    private final Pose endPose = new Pose(95.1028, 93.9813, Math.toRadians(37));
    private final Pose intake1 = new Pose(117.98130841121495, 83.4392523364486, Math.toRadians(0));

    private DcMotorEx leftFlywheel, rightFlywheel;
    private DualMotor flywheel;

    private Servo servo1, servo2, servo3;
    private DcMotorEx intake;

    private int pathState = 0;
    private Paths paths;

    @Override
    public void init() {
        // --- Flywheel ---
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel = new DualMotor(leftFlywheel,rightFlywheel);
        flywheel.setDirections(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);

        // --- follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        mecanumConstants = Constants.driveConstants;
        mecanumConstants.setMaxPower(0.6);

        // --- turret ---
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new AprilTagTurretControllerRed(hardwareMap);

        // --- servos ---
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");

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
        double power = turretController.getTurretPower(currentAngleDeg);
        turret.setPower(power);

        // --- path updates and shooting ---
        pathState = paths.autonomousPathUpdate(pathState, robotPose);

        // --- telemetry ---
        telemetry.addData("Robot Pose", robotPose);
        telemetry.addData("Turret Angle", currentAngleDeg);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Vision Lock", turretController.isLocked());
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    public class Paths {

        private PathChain path1, path2, path3;
        private Follower follow;
        private static final double POSITION_TOLERANCE = 1.5;
        private final double HEADING_TOLERANCE = Math.toRadians(2);

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
                new BezierCurve(
                    new Pose(95.103, 93.981),
                    new Pose(70.654, 78.729),
                    intake1
                )
            )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
                .build();
            path3 = follower
                .pathBuilder()
                .addPath(
                    new BezierCurve(
                        intake1,
                        new Pose(70.654, 78.729),
                        endPose
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))
                .build();
        }

        public int autonomousPathUpdate(int pathState, Pose robotPose) {
            switch (pathState) {

                case 0:
                    follow.followPath(path1);
                    flywheel.setPower(1);
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
                    if (pathTimer.getElapsedTimeSeconds() > 2){
                        servo1.setPosition(0.76);
                        if (pathTimer.getElapsedTimeSeconds() > 3){
                            servo1.setPosition(0);
                            pathTimer.resetTimer();
                            pathState = 3;
                    }
                    }
                    break;

                case 3:
                    servo2.setPosition(0);
                    if (pathTimer.getElapsedTimeSeconds() > 2){
                        servo2.setPosition(1);
                        pathTimer.resetTimer();
                        pathState = 4;
                    }
                    break;

                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 1){
                        servo3.setPosition(0.76);
                        if (pathTimer.getElapsedTimeSeconds() > 2){
                            servo3.setPosition(0);
                            pathTimer.resetTimer();
                            pathState = 5;
                            intake.setPower(-1);
                    }
                    }
                    break;

                case 5:
                   follow.followPath(path2);
                   pathState = 6;
                    break;
                case 6:
                    if (follow.atPose(intake1,2,2)){
                        pathState = 7;
                    }
                case 7:
                    follow.followPath(path3);
                    pathState = 8;
                    break;
                case 8:
                    if (follow.atPose(endPose,2,2)){
                        pathTimer.resetTimer();
                        pathState = 2;
                    }
                    break;
            }
            return pathState;
        }
    }
}
