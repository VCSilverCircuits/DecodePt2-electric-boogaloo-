package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTagControllers.TestingTurretControllerRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Test TeleOp RED")
public class TurretTestingTeleOp extends OpMode {

    /* ================= DRIVE ================= */
    private Follower follower;

    /* ================= TURRET ================= */
    private DcMotorEx turret;
    private TestingTurretControllerRed turretController;

    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_REV = 537.6;
    private static final double DEG_PER_TICK =
        360.0 / TICKS_PER_REV / BELT_RATIO;

    /* Motor-side control */
    private static final double TURRET_P = 0.08;
    private static final double MAX_TURRET_POWER = 0.8;

    private boolean manualMode = false;
    private double targetAngleDeg = 0;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        follower = Constants.createFollower(hardwareMap);

        // 🔴 SET THIS CORRECTLY
        follower.setStartingPose(
            new Pose(84.71028037383176, 9.121495327102796, Math.toRadians(0))
        );
        follower.update();

        turretController = new TestingTurretControllerRed(hardwareMap);
        turretController.resetController();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        double turretAngleDeg =
            turret.getCurrentPosition() * DEG_PER_TICK;

        // Zero turret controller
        turretController.setPosition(0, turretAngleDeg);
    }

    @Override
    public void loop() {

        /* ================= DRIVE ================= */
        follower.update();

        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );

        Pose pose = follower.getPose();

        /* Feed Pedro pose into turret controller */
        turretController.updateRobotPose(
            pose.getX(),
            pose.getY(),
            pose.getHeading()
        );

        /* ================= MODE SWITCH ================= */
        if (gamepad2.a && !manualMode) {
            manualMode = true;
            turretController.resetController();
        }

        if (gamepad2.b && manualMode) {
            manualMode = false;
            turretController.resetController();
        }

        double turretAngleDeg =
            turret.getCurrentPosition() * DEG_PER_TICK;

        double power;

        /* ================= TURRET CONTROL ================= */
        if (manualMode) {
            power = gamepad2.left_stick_x * 0.4;
        } else {

            targetAngleDeg =
                turretController.getTargetAngle(turretAngleDeg);

            double error =
                AngleUnit.normalizeDegrees(targetAngleDeg - turretAngleDeg);

            power = error * TURRET_P;
            power = Math.max(-MAX_TURRET_POWER,
                Math.min(MAX_TURRET_POWER, power));
        }

        turret.setPower(power);

        /* ================= TELEMETRY ================= */
        telemetry.addData("Mode", manualMode ? "MANUAL" : "AUTO");
        telemetry.addData("Turret Angle (deg)", turretAngleDeg);
        telemetry.addData("Target Angle (deg)", targetAngleDeg);
        telemetry.addData("Has Tag Lock", turretController.hasLock());
        telemetry.addData("Robot X", pose.getX());
        telemetry.addData("Robot Y", pose.getY());
        telemetry.update();
    }
}
