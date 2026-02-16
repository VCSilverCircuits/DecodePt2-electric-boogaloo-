package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Shooter RPM Tuning")
public class LookUpTableTesting extends OpMode {

    // ================= FLYWHEEL =================
    private DcMotorEx leftFlywheel, rightFlywheel;

    private double TICKS_PER_REV = 36.13;


    private double targetRPM = 3500;   // starting RPM
    private static final double RPM_INCREMENT = 50;

    private boolean flywheelToggle = false;
    private boolean lastFlywheelTrigger = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set PIDF once
        leftFlywheel.setVelocityPIDFCoefficients(0.013, 0, 0, 12.9);
        rightFlywheel.setVelocityPIDFCoefficients(0.013, 0, 0, 12.9);
    }

    @Override
    public void loop() {
        // ================= TOGGLE FLYWHEEL =================
        boolean flywheelPressed = gamepad2.right_trigger > 0.5;
        if (flywheelPressed && !lastFlywheelTrigger) {
            flywheelToggle = !flywheelToggle;
        }
        lastFlywheelTrigger = flywheelPressed;

        // ================= MANUAL RPM CONTROL =================
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;

        if (dpadUp && !lastDpadUp) {
            targetRPM += RPM_INCREMENT;
        }

        if (dpadDown && !lastDpadDown) {
            targetRPM -= RPM_INCREMENT;
        }

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;

        // Clamp RPM so you don’t blow hardware
        targetRPM = Math.max(0, Math.min(targetRPM, 6500));

        if (flywheelToggle) {
            setFlywheelRPM(targetRPM);
        } else {
            stopFlywheel();
        }

        // ================= TELEMETRY =================
        telemetry.addLine("=== SHOOTER TUNING MODE ===");
        telemetry.addData("Raw Velocity (ticks/sec)", leftFlywheel.getVelocity());
        telemetry.addData("Flywheel Enabled", flywheelToggle);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", getCurrentRPM());
        telemetry.addData("Error", targetRPM - getCurrentRPM());
        telemetry.update();
    }

    private void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

    private void stopFlywheel() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    private double getCurrentRPM() {
        double ticksPerSecond = rightFlywheel.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
}
