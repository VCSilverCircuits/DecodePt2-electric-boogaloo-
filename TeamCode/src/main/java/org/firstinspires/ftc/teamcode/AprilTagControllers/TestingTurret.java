package org.firstinspires.ftc.teamcode.AprilTagControllers;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Everything is in RADIANS
@Configurable
public class TestingTurret {
    DcMotorEx yawMotor;
    Follower follower;
    Limelight3A limelight;
    double MAX_TURRET_RAD = Math.toRadians(175);
    double MIN_TURRET_RAD = Math.toRadians(-90 );

    double turretPosition;

    double isLLgetting;

    // TODO: TUNE THIS VALUE!
    public static double RADIANSPERTICK = 0.001062;

    final Pose REDTARGET = new Pose(137.0, 143.0);
    final Pose BLUETARGET = new Pose(6.0, 143.0);

    // TODO: Tune these. Expect very different P values!
    final PIDFController limelightPIDF = new PIDFController(0.0, 0.0, 0.00, 0.0);
    final PIDFController odometryPIDF = new PIDFController(1.1, 0.0, 0.003, 0.2);

    double relativeTargetHeading;
    boolean isRed;

    /**
     *
     * @param hardwareMap Used to retrieve hardware from configuration file in driver hub
     * @param follower    Used as fallback to determine distance to target using odometry
     * @param isRed       Set per alliance color
     */
    public TestingTurret(HardwareMap hardwareMap, Follower follower, boolean isRed) {
        // Stores follower and alliance color
        this.follower = follower;
        this.isRed = isRed;

        // Declares and sets up turret servos
        yawMotor = hardwareMap.get(DcMotorEx.class, "turretRotation");
        yawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Declares and sets up limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();
    }

    /**
     * Calculates relative position of the target using odometry
     */
    public void update() {
        // Calculate turret angle relative to the robot chassis
        turretPosition = AngleUnit.normalizeRadians(yawMotor.getCurrentPosition() * RADIANSPERTICK);

        // Get target coordinates
        double targetX = (isRed ? REDTARGET.getX() : BLUETARGET.getX());
        double targetY = (isRed ? REDTARGET.getY() : BLUETARGET.getY());

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getHeading();

        // Calculate the field-centric angle to the target
        double fieldAngleToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Convert to robot-centric angle (where the turret needs to point)
        // Angle Wrapping: ensures we calculate the shortest distance (-PI to PI)
        relativeTargetHeading = AngleUnit.normalizeRadians(fieldAngleToTarget - robotHeading);
    }

    /**
     * Aim using limelight or odometry as fallback
     */
    public void aim() {
        limelight.start();
        // Retrieve limelight data
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Limelight tx is in degrees. Target is 0.
            double power = limelightPIDF.calculate(llResult.getTx(), 0);
            // power = normalizePower(power);
            isLLgetting = power;
            yawMotor.setPower(power);
        } else {
            // Fallback to Odometry
            // We want turretPosition to match relativeTargetHeading
            double power = odometryPIDF.calculate(turretPosition, relativeTargetHeading);
            power = normalizePower(power);

            yawMotor.setPower(power);
        }
    }
    public void odoAim() {
        double targetAngle = relativeTargetHeading;

        // Check if target is outside physical limits
        if (targetAngle >= MAX_TURRET_RAD) {
            // Flip to opposite direction
            targetAngle = AngleUnit.normalizeRadians(targetAngle + Math.PI);
        } else if (targetAngle <= MIN_TURRET_RAD) {
            // Flip to opposite direction
            targetAngle = AngleUnit.normalizeRadians(targetAngle + Math.PI/2);
        }

        // PID calculation
        double power = odometryPIDF.calculate(turretPosition, targetAngle);

        // Clamp and set power
        yawMotor.setPower(Math.max(-1, Math.min(1, power)));
    }


    /**
     * Stops turret and limelight when not aiming
     */
    public void idle() {
        // Keep turret centered forward (position 0 relative to robot)
        double power = normalizePower(odometryPIDF.calculate(turretPosition, 0));
        yawMotor.setPower(power);
        limelight.pause();
    }

    public void adjustTurret(double power) {
        yawMotor.setPower(power);
    }

    /**
     * Avoid using this, use idle() method instead
     */
    public void stop() {
        yawMotor.setPower(0);
        limelight.stop();
    }

    private double normalizePower(double power) {
        return Math.max(-1, Math.min(1, power));
    }


    public double getTurretPosition() {
        return yawMotor.getCurrentPosition();
    }

    public double getRelativeTargetHeading() {
        return relativeTargetHeading;
    }

    public double checkLL() {
        return isLLgetting;
    }
}

