package org.firstinspires.ftc.teamcode.Subsystems;

import android.health.connect.datatypes.units.Velocity;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlywheelConstants {

    private DcMotorEx leftFlywheel, rightFlywheel;
    private Servo hoodServo;
    PIDFController VelocityPIDF;

    public static final double TICKS_PER_REV = 28;

    // ================= LINEAR REGRESSION CONSTANTS =================
    // RPM Regression
    private static final double RPM_SLOPE = 21.35148;
    private static final double RPM_INTERCEPT = 2434.33841;

    // Hood Regression (REPLACE WITH YOUR ACTUAL VALUES)
    private static final double HOOD_SLOPE = -0.880766;      // example
    private static final double HOOD_INTERCEPT = 139.12875;    // example

    private final double velocityCompGain = 2.0;
    private final double goalX;
    private final double goalY;
    private Follower follower;

    private double robotX;
    private double robotY;

    private double targetRPM = 0;
    private double targetHoodAngle = 0;
    private boolean enabled = false;

    public FlywheelConstants(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        hoodServo = hardwareMap.get(Servo.class, "Hood");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        VelocityPIDF = new PIDFController(0.0085, 0.003, 0, 0.0);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        goalX = 144;
        goalY = 144;
    }

    public void update(double robotForwardVelocity) {
        if (!enabled) {
            stop();
            return;
        }

        robotX = follower.getPose().getX();
        robotY = follower.getPose().getY();

        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.hypot(dx, dy);

        // ===== Regression-Based Calculations =====
        double baseRPM = getRegressionRPM(distance);
        double baseHood = getRegressionHood(distance);

        targetRPM = baseRPM + velocityCompGain * robotForwardVelocity;
        targetHoodAngle = baseHood;

        applyRPM(targetRPM);
        applyHoodAngle(targetHoodAngle);
    }

    // ================= RPM REGRESSION =================
    private double getRegressionRPM(double distance) {
        double rpm = RPM_SLOPE * distance + RPM_INTERCEPT;

        // Clamp to safe range
        return Math.max(1800, Math.min(6000, rpm));
    }

    // ================= HOOD REGRESSION =================
    private double getRegressionHood(double distance) {
        double angle = HOOD_SLOPE * distance + HOOD_INTERCEPT;

        // Clamp to mechanical limits
        return Math.max(0, Math.min(120, angle));
    }

    // ================= MOTOR CONTROL =================
    private void applyRPM(double rpm) {
        double power = VelocityPIDF.calculate(getCurrentRPM(), targetRPM);
        leftFlywheel.setPower(power);
        rightFlywheel.setPower(power);
    }

    private void applyHoodAngle(double angleDegrees) {
        double minAngle = 0;
        double maxAngle = 120;

        double position = (angleDegrees - minAngle) / (maxAngle - minAngle);
        position = Math.max(0, Math.min(1, position));

        hoodServo.setPosition(position);
    }

    private void stop() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    // ================= GETTERS =================
    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        double ticksPerSecond = rightFlywheel.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public boolean atSpeed(double tolerance) {
        return Math.abs(getCurrentRPM() - targetRPM) < tolerance;
    }
}
