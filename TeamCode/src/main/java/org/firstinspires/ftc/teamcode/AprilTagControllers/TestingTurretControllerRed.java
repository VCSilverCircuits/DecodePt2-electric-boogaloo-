package org.firstinspires.ftc.teamcode.AprilTagControllers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class TestingTurretControllerRed {

    private static final int TARGET_ID = 24;

    // ===== VISION PID (ANGLE DOMAIN) =====
    private double kP = 0.04;
    private double kD = 0.002;

    private static final double DEADBAND_DEG = 1.0;

    // ===== TURRET LIMITS =====
    private static final double MIN_ANGLE_DEG = -80;
    private static final double MAX_ANGLE_DEG = 260;

    private final Limelight3A limelight;

    // ===== PID STATE =====
    private double lastError = 0;
    private long lastTimeNs = 0;

    // ===== TARGET STATE =====
    private double lastTargetAngleDeg = 0;
    private boolean hasLock = false;

    // ===== ZEROING =====
    private double turretZeroOffsetDeg = 0;
    private boolean turretZeroed = false;

    public TestingTurretControllerRed(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Define what encoder angle corresponds to a known field angle
     */
    public void setPosition(double desiredAngleDeg, double currentTurretAngleDeg) {
        turretZeroOffsetDeg = desiredAngleDeg - currentTurretAngleDeg;
        turretZeroed = true;
    }

    /**
     * Returns the desired turret angle (degrees).
     * Encoder PID will handle motor control.
     */
    public double getTargetAngle(double currentTurretAngleDeg) {

        if (turretZeroed) {
            currentTurretAngleDeg += turretZeroOffsetDeg;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == TARGET_ID) {
                    hasLock = true;

                    double tx = result.getTx(); // degrees yaw error
                    double correction = visionPid(tx);

                    lastTargetAngleDeg =
                        clamp(currentTurretAngleDeg + correction,
                            MIN_ANGLE_DEG, MAX_ANGLE_DEG);

                    return lastTargetAngleDeg;
                }
            }
        }

        // No tag → hold last known target
        hasLock = false;
        return lastTargetAngleDeg;
    }

    // ===== VISION PID (ANGLE OUTPUT) =====
    private double visionPid(double errorDeg) {

        if (Math.abs(errorDeg) < DEADBAND_DEG) {
            return 0;
        }

        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0 : (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        double derivative = (dt > 0) ? (errorDeg - lastError) / dt : 0;
        lastError = errorDeg;

        return kP * errorDeg + kD * derivative;
    }

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
        lastError = 0;
        lastTimeNs = 0;
        hasLock = false;
    }

    // Optional setters for live tuning later
    public void setVisionP(double p) { kP = p; }
    public void setVisionD(double d) { kD = d; }
}
