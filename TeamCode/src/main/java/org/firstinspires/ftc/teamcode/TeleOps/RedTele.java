package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensor1Test;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Red TeleOp")
public class RedTele extends OpMode {
    ColorSensor1Test colorSensor1Test;

    // ================= DRIVE =================
    private Follower follower;

    // ================= TURRET =================
    private DcMotorEx turret;
    private AprilTagTurretControllerRed turretController;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    // ================= SHOOTER =================
    private DcMotorEx leftFlywheel, rightFlywheel;
    private Servo hoodServo;
    private double targetRPM = 0;
    private double hoodPosition = 0.5;
    private static final double RPM_INCREMENT = 50;
    private static final double HOOD_INCREMENT = 0.01;
    private static final double MAX_MOTOR_RPM = 5800.0;
    private static final double GEAR_RATIO = 40.0 / 100.0;
    private static final double TICKS_PER_REV = 537.6;

    // ================= FLIPPERS =================
    private Servo servo1, servo2, servo3;

    // ================= INTAKE =================
    private DcMotorEx intake;
    private boolean intakeToggle = false;
    private boolean lastRT = false;

    @Override
    public void init() {

        // --- turret ---
        turretController = new AprilTagTurretControllerRed(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // --- flywheel / hood ---
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        hoodServo = hardwareMap.get(Servo.class, "shooterAngler");

        // --- intake ---
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // --- flippers ---
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");

        // --- drive follower ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(83.44, 15.25, Math.toRadians(0))); // example start
        follower.update();

        // --- hood start ---
        hoodServo.setPosition(hoodPosition);
        servo2.setPosition(1);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        follower.update();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );

        // Current robot pose for odometry
        Pose robotPose = follower.getPose();

        // Current turret angle
        double currentTurretAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;

        // ================= TURRET CONTROL =================
        double turretPower = turretController.getTurretPower(currentTurretAngleDeg, robotPose, 0);
        turret.setPower(turretPower);

        // ================= FLYWHEEL =================
        if (gamepad1.right_bumper) targetRPM += RPM_INCREMENT;
        if (gamepad1.left_bumper) targetRPM -= RPM_INCREMENT;
        targetRPM = Math.max(0, Math.min(MAX_MOTOR_RPM, targetRPM));

        double flywheelPower = Math.min(targetRPM / MAX_MOTOR_RPM, 1.0);
        leftFlywheel.setPower(flywheelPower);
        rightFlywheel.setPower(flywheelPower);

        // ================= HOOD =================
        if (gamepad1.dpad_right) hoodPosition += HOOD_INCREMENT;
        if (gamepad1.dpad_left)  hoodPosition -= HOOD_INCREMENT;
        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));
        hoodServo.setPosition(hoodPosition);

        // ================= FLIPPERS =================
        servo1.setPosition(gamepad1.a ? 0.79 : 0);
        if (gamepad1.b){
            servo2.setPosition(0);
        } else {
            servo2.setPosition(1);
        }
        servo3.setPosition(gamepad1.x ? 0.79 : 0);

        // ================= INTAKE =================
        boolean rtPressed = gamepad1.right_trigger > 0.5;
        if (rtPressed && !lastRT) intakeToggle = !intakeToggle;
        lastRT = rtPressed;
        intake.setPower(intakeToggle ? -1 : 0);

        // ================= TELEMETRY =================
        telemetry.addData("Turret Angle", currentTurretAngleDeg);
        telemetry.addData("Vision Lock", turretController.isLocked());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Using Mode", turretController.isLocked() ? "Vision" : "Odometry");
        telemetry.addData("servo", servo2.getPosition());
        telemetry.update();
    }
}
