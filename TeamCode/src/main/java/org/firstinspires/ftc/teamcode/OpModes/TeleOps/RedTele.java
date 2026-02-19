package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red Tele")
public class RedTele extends OpMode {

    private FlywheelConstants flywheel;
    private Follower follower;
    private OdoAim turret;
    private ServoGroup servos;
    private ColorSensors sensors;

    private DcMotorEx intake;

    private Servo lift1, lift2;
    private Servo stopper;

    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    private boolean liftToggle = false;
    private boolean lastLiftToggle = false;

    private boolean isFiring = false;
    private boolean intakeLocked = false;   // 🔒 NEW LOCK VARIABLE

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new FlywheelConstants(hardwareMap, follower, true);

        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(
            hardwareMap,
            "frontFlipper",
            "backFlipper",
            "leftFlipper"
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
        follower.setStartingPose(new Pose(84.037, 10.019, Math.toRadians(0)));
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
        turret.update();
        turret.odoAim();
        servos.loop();

        // ================= FIRING LOGIC =================
        if (gamepad1.y && !isFiring) {

            isFiring = true;
            intakeLocked = true;  // 🔒 LOCK INTAKE
            intake.setPower(0);

            sensors.reset();
            sensors.update();

            servos.StartNonSort();
        }

        // Automatically unlock when done
        if (!servos.isRunning() && isFiring) {
            isFiring = false;
            intakeLocked = false;  // 🔓 UNLOCK INTAKE
        }

        // Stopper logic
        if (servos.isRunning()) {
            stopper.setPosition(0.3);
        } else {
            stopper.setPosition(0);
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
            intake.setPower(0);  // Force off if locked
        }

        // ================= LIFT TOGGLE =================
        boolean frontPressed = gamepad2.left_trigger > 0.5;

        if (frontPressed && !lastLiftToggle) {
            liftToggle = !liftToggle;
        }
        lastLiftToggle = frontPressed;

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
        telemetry.addData("Intake Locked", intakeLocked);
        telemetry.addData("Is Firing", isFiring);
        telemetry.update();
    }
}
