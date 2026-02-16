package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlywheelConstants {

    private DcMotorEx leftFlywheel, rightFlywheel;
  //  private Servo hoodServo;

    public static final double TICKS_PER_REV = 28;

    // ================= LOOKUP TABLES =================
    private final double[] distances = {22, 38.5, 58.5, 75.5, 97, 111};
    private final double[] rpms      = {2623, 2789, 3022, 3155, 3520, 4350};
    //private final double[] hoodAngles = {10, 15, 22, 28, 35, 40}; // degrees

    private final double velocityCompGain = 2.0;
    private final double goalX;
    private final double goalY;
    private Follower follower;

    // Placeholder robot state
    private double robotX;
    private double robotY;

    private double targetRPM = 0;
    private double targetHoodAngle = 0;
    private boolean enabled = false;

    public FlywheelConstants(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
       // hoodServo = hardwareMap.get(Servo.class, "Hood"); // servo name in config

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftFlywheel.setVelocityPIDFCoefficients(0.027, 0, 0.004, 12.9);
        rightFlywheel.setVelocityPIDFCoefficients(0.027, 0, 0.004, 12.9);

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

        double baseRPM = getInterpolatedRPM(distance);
        targetRPM = (baseRPM + velocityCompGain * robotForwardVelocity) * 1.2;

       // targetHoodAngle = getInterpolatedHoodAngle(distance);

        applyRPM(targetRPM);
       // applyHoodAngle(targetHoodAngle);
    }

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

    private void applyRPM(double rpm) {
        double ticksPerSecond = ((rpm * TICKS_PER_REV) / 55.0) *1.2;

        leftFlywheel.setVelocity(ticksPerSecond);
        rightFlywheel.setVelocity(ticksPerSecond);
    }

   /* private void applyHoodAngle(double angleDegrees) {
        // Convert angle to servo position (0-1)
        // Example: 10° -> 0.0, 40° -> 1.0 (linear mapping)
        double minAngle = 10;
        double maxAngle = 40;

        double position = (angleDegrees - minAngle) / (maxAngle - minAngle);
        position = Math.max(0, Math.min(1, position)); // clamp 0-1
        hoodServo.setPosition(position);
    }*/

    private void stop() {
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    private double getInterpolatedRPM(double distance) {
        if (distance <= distances[0])
            return rpms[0];
        if (distance >= distances[distances.length - 1])
            return rpms[rpms.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double x1 = distances[i];
                double x2 = distances[i + 1];
                double y1 = rpms[i];
                double y2 = rpms[i + 1];
                return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
            }
        }
        return rpms[0];
    }

  /*  private double getInterpolatedHoodAngle(double distance) {
        if (distance <= distances[0])
            return hoodAngles[0];
        if (distance >= distances[distances.length - 1])
            return hoodAngles[hoodAngles.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double x1 = distances[i];
                double x2 = distances[i + 1];
                double y1 = hoodAngles[i];
                double y2 = hoodAngles[i + 1];
                return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
            }
        }
        return hoodAngles[0];
    }*/

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        double ticksPerSecond = rightFlywheel.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

   /* public double getTargetHoodAngle() {
        return targetHoodAngle;
    }*/

    public boolean atSpeed(double tolerance) {
        return Math.abs(getCurrentRPM() - targetRPM) < tolerance;
    }
}
