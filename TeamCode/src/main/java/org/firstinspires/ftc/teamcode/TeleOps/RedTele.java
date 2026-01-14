package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AprilTagControllers.AprilTagTurretControllerRed;
import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensor1Test;
import org.firstinspires.ftc.teamcode.DualMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="Red TeleOp")
public class RedTele extends OpMode {
    ColorSensor1Test colorSensor1Test;
    AprilTagTurretControllerRed turretController;

    // ================= DRIVE =================
    private Follower follower;

    // ================= TURRET =================
    private DcMotorEx turret;
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_MOTOR_REV = 537.6;
    private static final double DEGREES_PER_TICK =
        360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;
    public boolean lastRightTriggeredPressed = false;
    public boolean lastLeftTriggeredPressed = false;
    private boolean last2LT = false;

    // ================= SHOOTER =================
    private DcMotorEx leftFlywheel, rightFlywheel;
    private Servo hoodServo;
    private double targetRPM = 0;
    private double hoodPosition = 0.5;
    private static final double HOOD_INCREMENT = 0.01;
    DualMotor flywheel = new DualMotor(leftFlywheel, rightFlywheel);
    private boolean flywheelToggle;


    // ================= FLIPPERS =================
    private Servo servo1, servo2, servo3;

    // ================= INTAKE =================
    private DcMotorEx intake;
    private boolean intakeToggle = false;
    private boolean backspinToggle = false;


    @Override
    public void init() {

        // --- turret ---
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
        flywheel = new DualMotor(leftFlywheel, rightFlywheel);
        flywheel.setDirections(
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.FORWARD
        );

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

        turretController = new AprilTagTurretControllerRed(hardwareMap);
        turretController.resetController();


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        turretController.setPosition(0, currentAngleDeg);
    }

    @Override
    public void loop() {
        double currentAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
        double power = turretController.getTurretPower(currentAngleDeg);
        turret.setPower(power);
        // ================= DRIVE =================
        follower.update();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );
        // Current turret angle


        turret.setPower((gamepad2.left_stick_x / 5) * 3);


        // ================= FLYWHEEL =================
        boolean rtPressed = gamepad2.right_trigger > 0.5;
        if (rtPressed && !lastRightTriggeredPressed) flywheelToggle = !flywheelToggle;
        lastRightTriggeredPressed = rtPressed;
        flywheel.setPower(flywheelToggle ? 1 : 0);


        // ================= FLIPPERS =================
        servo1.setPosition(gamepad1.b ? 0.79 : 0);
        if (gamepad1.a) {
            servo2.setPosition(0);
        } else {
            servo2.setPosition(1);
        }
        servo3.setPosition(gamepad1.x ? 0.79 : 0);
        // ================= INTAKE AND BACKSPIN =================
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

// Toggle intake with left trigger
        if (leftTriggerPressed && !lastLeftTriggeredPressed) intakeToggle = !intakeToggle;
        lastLeftTriggeredPressed = leftTriggerPressed;

// Toggle backspin with right trigger
        if (rightTriggerPressed && !lastRightTriggeredPressed) backspinToggle = !backspinToggle;
        lastRightTriggeredPressed = rightTriggerPressed;

// Set motor power based on toggles
        if (intakeToggle) {
            intake.setPower(1);       // Intake spins forward
        } else if (backspinToggle) {
            intake.setPower(-1);      // Backspin spins backward
        } else {
            intake.setPower(0);       // Stop motor if neither
        }

// Set intake power based on toggles
            // ================= TELEMETRY =================

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Hood Position", hoodPosition);
            telemetry.addData("servo", servo2.getPosition());
            telemetry.addData("Turret Encoder", turret.getCurrentPosition());
            telemetry.addData("Turret Angle (deg)", currentAngleDeg);
            telemetry.addData("Turret Power", power);
            telemetry.addData("turretPosition", turret.getCurrentPosition());
            telemetry.update();
        }
    }
