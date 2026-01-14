package org.firstinspires.ftc.teamcode.AprilTagControllers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.geometry.Pose;

import java.util.List;

public class AprilTagTurretControllerBlue {

    // ================= CONFIG =================
    private static final int TARGET_ID = 20;

    // Field coordinates of goal (PedroPathing frame)
    private static final double GOAL_X = 130.99065420560748;   // inches
    private static final double GOAL_Y = 135.70093457943926;  // inches

    // PIDF tuning
    private static final double kP = 0.035;
    private static final double kI = 0.0;
    private static final double kD = 0.004;

    private static final double MAX_POWER = 0.6;
    private static final double DEADBAND = 1.0; // degrees

    // Soft limits
    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 270.0;

    // ================= HARDWARE =================
    private final Limelight3A limelight;

    // ================= STATE =================
    private boolean hasLock = false;
    private boolean hasEverSeenTag = false;

    // PID state (shared for vision & odometry)
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    // ================= CONSTRUCTOR =================
    public AprilTagTurretControllerBlue(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }

    // =================================================
    // ================= MAIN ENTRY ====================
    // =================================================
    /**
     * Computes turret motor power.
     * @param currentTurretAngleDeg Absolute turret angle (deg)
     * @param robotPose Current robot pose from odometry
     * @param unusedManualX Set to 0 for fully autonomous
     */
    public double getTurretPower(
        double currentTurretAngleDeg,
        Pose robotPose,
        double unusedManualX
    ) {

        // ================= 1️⃣ VISION =================
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_ID) {
                    hasLock = true;
                    hasEverSeenTag = true;
                    double tx = result.getTx(); // horizontal error in degrees
                    return pidController(tx);
                }
            }
        }

        // ================= 2️⃣ ODOMETRY =================
        hasLock = false;
        resetPID();

        if (robotPose != null) {
            double targetAngle = computeTargetTurretAngle(robotPose);
            return anglePID(targetAngle, currentTurretAngleDeg);
        }

        // ================= 3️⃣ FAILSAFE =================
        return 0.0;
    }

    // =================================================
    // ================= VISION PID ====================
    // =================================================
    private double pidController(double error) {
        if (Math.abs(error) <= DEADBAND) {
            resetPID();
            return 0.0;
        }
        return computePID(error);
    }

    // =================================================
    // ================= ANGLE PID =====================
    // =================================================
    private double anglePID(double targetDeg, double currentDeg) {
        double error = targetDeg - currentDeg;

        // shortest-path wrap
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        if (Math.abs(error) <= DEADBAND) {
            resetPID();
            return 0.0;
        }

        return computePID(error);
    }

    // =================================================
    // ================= PID CORE ======================
    // =================================================
    private double computePID(double error) {

        long now = System.nanoTime();

        if (lastTimeNs == 0) {
            lastTimeNs = now;
            lastError = error;
            return 0.0;
        }

        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        integralSum += error * dt;
        integralSum = clamp(integralSum, -10, 10);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        return clamp(output, -MAX_POWER, MAX_POWER);
    }

    // =================================================
    // ================= GEOMETRY ======================
    // =================================================
    private double computeTargetTurretAngle(Pose robotPose) {

        double dx = GOAL_X - robotPose.getX();
        double dy = GOAL_Y - robotPose.getY();

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;

        // normalize [0,360)
        turretTargetDeg = (turretTargetDeg + 360) % 360;

        // clamp to soft limits
        return clamp(turretTargetDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    // =================================================
    // ================= UTIL ==========================
    // =================================================
    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTimeNs = 0;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public boolean isLocked() {
        return hasLock;
    }

    public boolean hasSeenTag() {
        return hasEverSeenTag;
    }
}
