package org.firstinspires.ftc.teamcode.AprilTagControllers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * Red-side turret controller
 * Vision-first (AprilTag), smooth Pedro odometry fallback
 * Outputs desired turret angle (degrees)
 */
public class TestingTurretControllerRed {

    /* ===================== TARGET ===================== */
    private static final int TARGET_ID = 24;

    /* ===================== RED GOAL (PEDRO COORDS) ===================== */
    private static final double RED_GOAL_X = 144;
    private static final double RED_GOAL_Y = 144;

    /* ===================== VISION PID ===================== */
    private double visionP = 0.08;
    private double visionI = 0.0;
    private double visionD = 0.008;

    private static final double VISION_DEADBAND_DEG = 1.0;

    /* ===================== ODOMETRY PID ===================== */
    private double odomP = 0.08;
    private double odomD = 0.008;

    /* ===================== TURRET LIMITS ===================== */
    private static final double MIN_ANGLE_DEG = -260;
    private static final double MAX_ANGLE_DEG = 80;

    /* ===================== HARDWARE ===================== */
    private final Limelight3A limelight;

    /* ===================== VISION PID STATE ===================== */
    private double visionIntegral = 0;
    private double lastVisionError = 0;
    private long lastVisionTimeNs = 0;

    /* ===================== ODOM PID STATE ===================== */
    private double lastOdomError = 0;
    private long lastOdomTimeNs = 0;

    /* ===================== STATE ===================== */
    private boolean hasLock = false;
    private double lastTargetAngleDeg = 0;

    /* ===================== ZEROING ===================== */
    private double turretZeroOffsetDeg = 0;
    private boolean turretZeroed = false;

    /* ===================== PEDRO POSE ===================== */
    private double robotX;
    private double robotY;
    private double robotHeadingRad;

    public TestingTurretControllerRed(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // RED pipeline
        limelight.start();
    }

    /* ===================== POSE UPDATE ===================== */

    /** Call every loop from Pedro follower */
    public void updateRobotPose(double x, double y, double headingRad) {
        robotX = x;
        robotY = y;
        robotHeadingRad = headingRad;
    }

    /* ===================== ZEROING ===================== */

    public void setPosition(double desiredAngleDeg, double currentTurretAngleDeg) {
        turretZeroOffsetDeg = desiredAngleDeg - currentTurretAngleDeg;
        turretZeroed = true;
    }

    /* ===================== MAIN API ===================== */

    /**
     * Returns desired turret angle in degrees
     */
    public double getTargetAngle(double currentTurretAngleDeg) {

        if (turretZeroed) {
            currentTurretAngleDeg += turretZeroOffsetDeg;
        }

        LLResult result = limelight.getLatestResult();

        /* ========== VISION PATH ========== */
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_ID) {
                    hasLock = true;

                    double tx = result.getTx(); // degrees
                    double correction = visionPID(tx);

                    lastTargetAngleDeg =
                        clamp(currentTurretAngleDeg + correction,
                            MIN_ANGLE_DEG, MAX_ANGLE_DEG);

                    return lastTargetAngleDeg;
                }
            }
        }

        /* ========== PEDRO ODOMETRY FALLBACK ========== */
        hasLock = false;

        double targetOdomAngle = computeOdometryAngleDeg();
        double odomError =
            AngleUnit.normalizeDegrees(targetOdomAngle - currentTurretAngleDeg);

        double correction = odomPID(odomError);

        lastTargetAngleDeg =
            clamp(currentTurretAngleDeg + correction,
                MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        return lastTargetAngleDeg;
    }

    /* ===================== ODOMETRY AIMING ===================== */

    private double computeOdometryAngleDeg() {
        double dx = RED_GOAL_X - robotX;
        double dy = RED_GOAL_Y - robotY;

        double fieldAngleRad = Math.atan2(dy, dx);
        double relativeRad = fieldAngleRad - robotHeadingRad;

        return Math.toDegrees(AngleUnit.normalizeRadians(relativeRad));
    }

    /* ===================== VISION PID ===================== */

    private double visionPID(double errorDeg) {

        if (Math.abs(errorDeg) < VISION_DEADBAND_DEG) {
            return 0;
        }

        long now = System.nanoTime();
        double dt = (lastVisionTimeNs == 0) ? 0 : (now - lastVisionTimeNs) / 1e9;
        lastVisionTimeNs = now;

        if (dt > 0) {
            visionIntegral += errorDeg * dt;
        }

        double derivative = (dt > 0)
            ? (errorDeg - lastVisionError) / dt
            : 0;

        lastVisionError = errorDeg;

        return visionP * errorDeg
            + visionI * visionIntegral
            + visionD * derivative;
    }

    /* ===================== ODOM PID ===================== */

    private double odomPID(double errorDeg) {

        long now = System.nanoTime();
        double dt = (lastOdomTimeNs == 0) ? 0 : (now - lastOdomTimeNs) / 1e9;
        lastOdomTimeNs = now;

        double derivative = (dt > 0)
            ? (errorDeg - lastOdomError) / dt
            : 0;

        lastOdomError = errorDeg;

        return odomP * errorDeg + odomD * derivative;
    }

    /* ===================== UTIL ===================== */

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public boolean hasLock() {
        return hasLock;
    }

    public boolean hasZeroed() {
        return turretZeroed;
    }

    public void resetController() {
        visionIntegral = 0;
        lastVisionError = 0;
        lastVisionTimeNs = 0;

        lastOdomError = 0;
        lastOdomTimeNs = 0;

        hasLock = false;
    }

    /* ===================== TUNING ===================== */

    public void setVisionP(double p) { visionP = p; }
    public void setVisionI(double i) { visionI = i; }
    public void setVisionD(double d) { visionD = d; }

    public void setOdomP(double p) { odomP = p; }
    public void setOdomD(double d) { odomD = d; }
}
