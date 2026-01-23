package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Disabled
@TeleOp(name = "Flywheel PIDF Tuner", group = "TeleOp")
public class FlywheelPIDFTuner extends OpMode {

    // ===== Hardware =====
    private DcMotorEx leftFlywheel, rightFlywheel;

    // ===== PIDF =====
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    // ===== Target =====
    private double targetRPM = 3000;

    // ===== Constants =====
    private static final double TICKS_PER_REV = 28;

    // ===== Button states =====
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastRB, lastLB, lastY, lastA;

    @Override
    public void init() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        applyPIDF();
    }

    @Override
    public void loop() {

        // ===== PID tuning =====
        if (gamepad1.dpad_up && !lastDpadUp) kP += 0.002;
        if (gamepad1.dpad_down && !lastDpadDown) kP -= 0.002;

        if (gamepad1.dpad_right && !lastDpadRight) kD += 0.1;
        if (gamepad1.dpad_left && !lastDpadLeft) kD -= 0.1;

        // ===== F tuning =====
        if (gamepad1.right_bumper && !lastRB) kF += 0.1;
        if (gamepad1.left_bumper && !lastLB) kF -= 0.1;

        // ===== Target RPM =====
        if (gamepad1.y && !lastY) targetRPM += 250;
        if (gamepad1.a && !lastA) targetRPM -= 250;

        targetRPM = Math.max(0, targetRPM);
        kP = Math.max(0, kP);
        kD = Math.max(0, kD);
        kF = Math.max(0, kF);

        applyPIDF();

        // ===== Flywheel control =====
        if (gamepad1.right_trigger > 0.5) {
            setRPM(targetRPM);
        } else if (gamepad1.left_trigger > 0.5) {
            stopFlywheel();
        }

        // ===== Save button states =====
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
        lastY = gamepad1.y;
        lastA = gamepad1.a;

        // ===== Telemetry =====
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Velocity RPM", getRPM());
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }

    // ===== Helpers =====
    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        leftFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        rightFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void setRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    private void stopFlywheel() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    private double getRPM() {
        return (leftFlywheel.getVelocity() * 60.0) / TICKS_PER_REV;
    }
}
