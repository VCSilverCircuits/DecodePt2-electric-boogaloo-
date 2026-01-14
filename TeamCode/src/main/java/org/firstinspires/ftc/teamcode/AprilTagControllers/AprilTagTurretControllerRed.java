package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class AprilTagTurretControllerRed {

    private static final int TARGET_ID = 24;

    // ================= PIDF TUNING =================
    private static final double kP = 0.035;
    private static final double kI = 0.000;
    private static final double kD = 0.004;
    private static final double kF = 0.0;

    private static final double MAX_POWER = 0.6;
    private static final double DEADBAND = 1.0; // degrees

    // ================= SEARCH TUNING =================
    private static final double SEARCH_POWER = 0.15;

    // ================= SOFT LIMITS =================
    private static final double MIN_ANGLE_DEG = 0.0;
    private static final double MAX_ANGLE_DEG = 270.0;

    // ================= HARDWARE =================
    private final Limelight3A limelight;

    // ================= STATE =================
    private boolean hasLock = false;
    private boolean hasEverSeenTag = false;

    private double lastSeenTurretAngleDeg = 0.0;
    private double searchDirection = 1.0;

    // ================= PID STATE =================
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    public AprilTagTurretController(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }

    /**
     * Call every loop.
     * @param currentTurretAngleDeg absolute turret angle (degrees)
     * @return motor power [-1, 1]
     */
    public double getTurretPower(double currentTurretAngleDeg) {

        LLResult result = limelight.getLatestResult();

        // ================= NO VALID RESULT =================
        if (result == null || !result.isValid()) {
            resetPID();
            hasLock = false;

            if (!hasEverSeenTag) {
                return searchPower(currentTurretAngleDeg);
            } else {
                return moveTowardLastSeen(currentTurretAngleDeg);
            }
        }

        // ================= TAG DETECTION =================
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() == TARGET_ID) {

                hasLock = true;
                hasEverSeenTag = true;

                double tx = result.getTx(); // horizontal error (deg)

                // store last known angle
                lastSeenTurretAngleDeg = currentTurretAngleDeg;

                return pidTrack(tx);
            }
        }

        // ================= TAG LOST =================
        resetPID();
        hasLock = false;

        if (!hasEverSeenTag) {
            return searchPower(currentTurretAngleDeg);
        } else {
            return moveTowardLastSeen(currentTurretAngleDeg);
        }
    }

    // ====================================================
    // ================= PID CONTROLLER ===================
    // ====================================================

    private double pidTrack(double error) {

        // deadband
        if (Math.abs(error) <= DEADBAND) {
            resetPID();
            return 0.0;
        }

        long now = System.nanoTime();

        if (lastTimeNs == 0) {
            lastTimeNs = now;
            lastError = error;
            return 0.0;
        }

        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        // integral with windup protection
        integralSum += error * dt;
        integralSum = clamp(integralSum, -10.0, 10.0);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output =
            (kP * error) +
                (kI * integralSum) +
                (kD * derivative) +
                (kF * Math.signum(error));

        return clamp(output, -MAX_POWER, MAX_POWER);
    }

    // ====================================================
    // ================= SEARCH LOGIC =====================
    // ====================================================

    private double searchPower(double currentTurretAngleDeg) {

        if (currentTurretAngleDeg >= MAX_ANGLE_DEG) {
            searchDirection = -1.0;
        } else if (currentTurretAngleDeg <= MIN_ANGLE_DEG) {
            searchDirection = 1.0;
        }

        return searchDirection * SEARCH_POWER;
    }

    private double moveTowardLastSeen(double currentTurretAngleDeg) {

        double error = lastSeenTurretAngleDeg - currentTurretAngleDeg;

        if (Math.abs(error) <= DEADBAND) {
            return 0.0;
        }

        return clamp(Math.signum(error) * SEARCH_POWER, -MAX_POWER, MAX_POWER);
    }

    // ====================================================
    // ================= UTILITIES ========================
    // ====================================================

    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTimeNs = 0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void reverseSearchDirection() {
        searchDirection *= -1.0;
    }

    public boolean isLocked() {
        return hasLock;
    }

    public boolean hasSeenTag() {
        return hasEverSeenTag;
    }

    public double getLastSeenAngle() {
        return lastSeenTurretAngleDeg;
    }
}
