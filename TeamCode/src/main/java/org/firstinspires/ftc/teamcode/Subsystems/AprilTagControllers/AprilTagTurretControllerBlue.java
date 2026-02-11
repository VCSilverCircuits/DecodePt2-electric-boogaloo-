package org.firstinspires.ftc.teamcode.Subsystems.AprilTagControllers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class AprilTagTurretControllerBlue {

    private static final int TARGET_ID = 20;

    // ===== PID =====
    private static final double kP = 0.08;
    private static final double kD = 0.008;
    private static final double MAX_POWER = 0.8;
    private static final double DEADBAND_DEG = 1;

    // ===== LIMITS =====
    private static final double MIN_ANGLE_DEG = -80;
    private static final double MAX_ANGLE_DEG = 260;

    private final Limelight3A limelight;

    // --- PID variables ---
    private double lastError = 0;
    private long lastTimeNs = 0;

    // --- Vision lock ---
    private boolean hasLock = false;

    // --- Turret zeroing ---
    private double turretZeroOffsetDeg = 0;
    private boolean turretZeroed = false;

    public AprilTagTurretControllerBlue(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Sets the current turret encoder position to be treated as a specific angle
     * @param desiredAngleDeg the angle you want to consider as "zeroed"
     * @param currentTurretAngleDeg the current encoder angle in degrees
     */
    public void setPosition(double desiredAngleDeg, double currentTurretAngleDeg) {
        turretZeroOffsetDeg = desiredAngleDeg - currentTurretAngleDeg;
        turretZeroed = true;
    }

    /**
     * Computes the turret power based on vision tracking
     * @param currentTurretAngleDeg turret encoder angle in degrees
     */
    public double getTurretPower(double currentTurretAngleDeg) {

        // Apply zero offset if set
        if (turretZeroed) {
            currentTurretAngleDeg += turretZeroOffsetDeg;
        }

        LLResult result = limelight.getLatestResult();

        // ==================== VISION MODE ====================
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_ID) {
                    hasLock = true;

                    double tx = result.getTx(); // degrees offset from limelight
                    return pid(tx);
                }
            }
        }

        // No tag visible → stop turret
        hasLock = false;
        return 0;
    }

    // ==================== PID ====================
    private double pid(double errorDeg) {
        if (Math.abs(errorDeg) < DEADBAND_DEG) return 0;

        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double derivative = (dt > 0) ? (errorDeg - lastError) / dt : 0;
        lastError = errorDeg;

        double output = kP * errorDeg + kD * derivative;
        return clamp(output, -MAX_POWER, MAX_POWER);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public boolean isLocked() {
        return hasLock;
    }

    public boolean hasZeroed() {
        return turretZeroed;
    }

    // Optional: reset PID entirely
    public void resetController() {
        lastError = 0;
        lastTimeNs = 0;
    }
}
