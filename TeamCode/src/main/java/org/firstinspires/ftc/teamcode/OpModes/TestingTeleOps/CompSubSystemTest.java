package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

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
@TeleOp(name = "Comprehensive Testing TeleOp")
public class CompSubSystemTest extends OpMode {
    private FlywheelConstants flywheel;
    private Follower follower;
    private OdoAim turret;
    private ServoGroup servos;
    ColorSensors sensors;
    private DcMotorEx intake;
    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;
    private Servo lift1, lift2;
    private Servo stopper;
    private boolean isFiring = false;
    private boolean liftToggle = false;
    private boolean lastLiftToggle = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new FlywheelConstants(hardwareMap, follower);
        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(
            hardwareMap,
            "frontFlipper", "backFlipper", "leftFlipper"
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
        follower.setStartingPose(new Pose(84.037, 10.019, Math.toRadians(0))); // example start
        flywheel.enable();

    }

    @Override
    public void loop() {
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
        // ================= INTAKE AND BACKSPIN =================
        boolean intakePressed = gamepad1.left_trigger > 0.5;
        boolean backspinPressed = gamepad1.right_trigger > 0.5;

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
        if (gamepad1.y && !isFiring) {
            // Latch current colors
            sensors.reset(); // clear previous latches
            sensors.update();
            // Start servo sequence based on motif + current sensor colors
            servos.StartNonSort();
        }
        if (!servos.isRunning()){
            isFiring = false;
        }
        if (!servos.isRunning()) {
            stopper.setPosition(0);
        } else {
            stopper.setPosition(0.3);
        }
        boolean frontPressed = gamepad2.left_trigger > 0.5;
        if (frontPressed && !lastLiftToggle) {
            liftToggle = !liftToggle;
        }
        lastLiftToggle = frontPressed;

        if (liftToggle) {
            lift2.setPosition(0.52);
            lift1.setPosition(0.5);
        } else {
            lift2.setPosition(0.92);
            lift1.setPosition(0.9);
        }
    }
}
