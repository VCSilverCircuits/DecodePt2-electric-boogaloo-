package org.firstinspires.ftc.teamcode.AprilTagControllers;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Turret handles rotation using PID control.
 * Fixed: TICKS_PER_DEGREE math and loop efficiency.
 */
public class Turret {

    private final Follower follower;
    private final DcMotorEx yawMotor;
    private final Limelight3A limelight;

    // PID Controllers
    private final PIDFController visionPID;
    private final PIDFController positionPID;

    // --- Tunable Constants ---
    public static double VISION_P = 0.06;
    public static double VISION_I = 0.0;
    public static double VISION_D = 0.005;
    public static double VISION_F = 0.0;

    public static double POS_P = 0.01;
    public static double POS_I = 0.0;
    public static double POS_D = 0.0001;
    public static double POS_F = 0.0;

    // --- Math Constants ---
    private static final double BELT_RATIO = 230.0 / 20.0; // 11.5
    private static final double TICKS_PER_MOTOR_REV = 537.7; // Common GoBILDA 19.2:1 (Verify yours!)
    // Corrected Math: (Total Ticks per turret rev) / 360 degrees
    private static final double TICKS_PER_DEGREE = (TICKS_PER_MOTOR_REV * BELT_RATIO) / 360.0;

    // State Variables
    private double currentPos;
    private double tx;
    private int lastPipeline = -1;

    // Debug Variables
    private double debugFieldAngleDeg = 0.0;
    private double debugRobotRelativeDeg = 0.0;
    private double debugTargetTicks = 0.0;

    public Turret(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;

        yawMotor = hardwareMap.get(DcMotorEx.class, "turretRotation");
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        visionPID = new PIDFController(VISION_P, VISION_I, VISION_D, VISION_F);
        positionPID = new PIDFController(POS_P, POS_I, POS_D, POS_F);
    }

    public void zeroEncoder() {
        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Centers the turret based on encoder ticks.
     */
    public void reset() {
        currentPos = yawMotor.getCurrentPosition();
        double power = positionPID.calculate(currentPos, 0);
        setSafeYawPower(power);
    }

    /**
     * Primary aim method.
     * Uses Limelight if target is visible, otherwise falls back to Odometry.
     */
    public void aim(boolean isRed) {
        // Only switch pipeline if it's different to save cycle time
        int targetPipeline = isRed ? 0 : 1;
        if (lastPipeline != targetPipeline) {
            limelight.pipelineSwitch(targetPipeline);
            lastPipeline = targetPipeline;
        }

        currentPos = yawMotor.getCurrentPosition();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            // Calculate power to bring tx to 0
            double power = visionPID.calculate(tx, 0);
            setSafeYawPower(power);
        } else {
            // ODOMETRY FALLBACK
            double targetX = isRed ? 137.0 : 0.0;
            double targetY = 143.0;

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeading = follower.getPose().getHeading();

            // Field-centric angle
            double fieldAngle = Math.atan2(targetY - robotY, targetX - robotX);
            // Robot-relative angle
            double relativeRadians = AngleUnit.normalizeRadians(fieldAngle - robotHeading);

            double targetTicks = radiansToTicks(relativeRadians);
            double power = positionPID.calculate(currentPos, targetTicks);
            setSafeYawPower(power);
        }
    }

    /**
     * Applies power with safety limits and a deadzone to stop oscillations.
     */
    private void setSafeYawPower(double power) {
        final double LEFT_LIMIT_DEG = -180;
        final double RIGHT_LIMIT_DEG = 90;

        double currentAngle = getCurrentTurretAngleDeg();

        // FORCE STOP if outside limits and trying to move further out
        if (currentAngle <= LEFT_LIMIT_DEG && power < 0) {
            yawMotor.setPower(0);
            return;
        }
        if (currentAngle >= RIGHT_LIMIT_DEG && power > 0) {
            yawMotor.setPower(0);
            return;
        }

        // Deadzone to prevent jitter
        if (Math.abs(power) < 0.02) {
            yawMotor.setPower(0);
            return;
        }

        // Cap power at 40% while we are debugging to prevent hardware damage
        double cappedPower = Math.max(-0.4, Math.min(power, 0.4));
        yawMotor.setPower(cappedPower);
    }
    public double radiansToTicks(double radians) {
        return Math.toDegrees(radians) * TICKS_PER_DEGREE;
    }

    public void updateOdometryDebug(boolean isRed) {
        double targetX = isRed ? 137.0 : 0.0;
        double targetY = 143.0;

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getPose().getHeading();

        double fieldAngle = Math.atan2(targetY - robotY, targetX - robotX);
        debugFieldAngleDeg = Math.toDegrees(fieldAngle);

        double relativeRadians = AngleUnit.normalizeRadians(fieldAngle - robotHeading);
        debugRobotRelativeDeg = Math.toDegrees(relativeRadians);
        debugTargetTicks = radiansToTicks(relativeRadians);
    }

    // Getters
    public double getTx() { return tx; }
    public double getCurrentPosition() { return currentPos; }
    public double getCurrentTurretAngleDeg() { return yawMotor.getCurrentPosition() / TICKS_PER_DEGREE; }
    public double getDebugFieldAngleDeg() { return debugFieldAngleDeg; }
    public double getDebugRobotRelativeDeg() { return debugRobotRelativeDeg; }
    public double getDebugTargetTicks() { return debugTargetTicks; }
}
