package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants.TeleFlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red Tele")
public class RedTele extends OpMode {

    private static final Pose startPose = new Pose(84.037, 10.019, Math.toRadians(0));

    private TeleFlywheelConstants flywheel;
    private Follower follower;
    private OdoAim turret;
    private ServoGroup servos;
    private ColorSensors sensors;

    private DcMotorEx intake;

    private Servo lift1, lift2;
    private Servo stopper;
    private Servo servo1, servo2, servo3;

    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    private boolean liftToggle = false;
    private boolean lastLiftToggle = false;

    private boolean isFiring = false;
    private boolean intakeLocked = false;

    // ================= TURRET OFFSET EDGE DETECTION =================
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new TeleFlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");

        servo1.setPosition(0.05);
        servo2.setPosition(0.05);
        servo3.setPosition(0.05);

        servos = new ServoGroup(
            hardwareMap,
            "frontFlipper",
            "backFlipper",
            "leftFlipper",
            "stopper"
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        stopper = hardwareMap.get(Servo.class, "stopper");

        lift1.setPosition(0.9);
        lift2.setPosition(0.92);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
        flywheel.enable();
    }

    @Override
    public void loop() {

        // ================= DRIVE =================
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );

        follower.update();
        flywheel.update(-gamepad1.left_stick_y * 50);

        // ================= TURRET UPDATE =================
        turret.update();

        // --------- D-PAD OFFSET CONTROL ----------
        if (gamepad1.dpad_left && !lastDpadLeft) {
            turret.adjustOffset(true, false);
        }

        if (gamepad1.dpad_right && !lastDpadRight) {
            turret.adjustOffset(false, true);
        }

        // Reset offset
        if (gamepad1.dpad_down && !lastDpadDown) {
            turret.resetOffset();
        }

        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;

        turret.odoAim();
        servos.loop();

        // ================= FIRING LOGIC =================
        if (gamepad1.y && !isFiring) {

            isFiring = true;
            intakeLocked = true;
            intake.setPower(0);

            sensors.reset();
            sensors.update();

            servos.StartNonSort();
        }

        if (!servos.isRunning() && isFiring) {
            isFiring = false;
            intakeLocked = false;
        }

        if (servos.isRunning()) {
            stopper.setPosition(0.3);
        } else {
            stopper.setPosition(0);
        }

        if (!isFiring) {
            servo1.setPosition(gamepad1.b ? 1 : 0);
            servo2.setPosition(gamepad1.a ? 1 : 0);
            servo3.setPosition(gamepad1.x ? 1 : 0);
        }

        // ================= INTAKE SYSTEM =================
        boolean intakePressed = gamepad1.left_trigger > 0.5;
        boolean backspinPressed = gamepad1.right_trigger > 0.5;

        if (!intakeLocked) {

            if (intakePressed && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
            }
            lastIntakeTrigger = intakePressed;

            if (backspinPressed) {
                intake.setPower(1);
            } else if (intakeToggle) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

        } else {
            intake.setPower(0);
        }

        // ================= LIFT TOGGLE =================
        boolean liftPressed = gamepad2.dpad_up;   // change button if you want

        if (liftPressed && !lastLiftToggle) {
            liftToggle = !liftToggle;
        }

        lastLiftToggle = liftPressed;

        if (liftToggle) {
            lift1.setPosition(0.5);
            lift2.setPosition(0.52);
        } else {
            lift1.setPosition(0.9);
            lift2.setPosition(0.92);
        }


        // ================= TELEMETRY =================
        telemetry.addData("Current RPM", flywheel.getCurrentRPM());
        telemetry.addData("Target RPM", flywheel.getTargetRPM());
        telemetry.addData("Turret Offset (deg)", turret.getOffsetDegrees());
        telemetry.addData("Relative Target (deg)", Math.toDegrees(turret.getRelativeTargetHeading()));
        telemetry.addData("Intake Locked", intakeLocked);
        telemetry.addData("Is Firing", isFiring);
        telemetry.update();
    }
}
