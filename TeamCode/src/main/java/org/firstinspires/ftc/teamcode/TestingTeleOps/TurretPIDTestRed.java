package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagControllers.TestingTurretControllerRed;

@TeleOp(name = "Turret PID Test (Angle-Based)")
public class TurretPIDTestRed extends OpMode {

    // ================= HARDWARE =================
    private DcMotorEx turret;
    private TestingTurretControllerRed visionController;

    // ================= TURRET CONSTANTS =================
    private static final double BELT_RATIO = 230.0 / 20.0;
    private static final double TICKS_PER_REV = 28;
    private static final double DEG_PER_TICK =
        360.0 / TICKS_PER_REV / BELT_RATIO;

    // ================= AUTO STEP SEQUENCE =================
    private double[] testAngles = {0, 30, -30, 60, -60, 0}; // angles to step through
    private int stepIndex = 0;
    private long lastStepTime = 0;
    private static final long STEP_DURATION_MS = 1500; // time to hold each step

    // ================= MODES =================
    enum Mode { ENCODER_ONLY, VISION_TARGET }
    private Mode mode = Mode.ENCODER_ONLY;

    enum Term { P, I, D, F }
    private Term selectedTerm = Term.P;

    // ================= ENCODER PID =================
    private double kP = 0.02;
    private double kI = 0.0;
    private double kD = 0.001;
    private double kF = 0.0;

    private double targetAngleDeg = 30;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    // ================= INPUT EDGE DETECTION =================
    private boolean lastA, lastUp, lastDown, lastLB, lastRB, lastX;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        visionController = new TestingTurretControllerRed(hardwareMap);
        visionController.resetController();
    }

    @Override
    public void loop() {

        // ================= MODE TOGGLE =================
        if (gamepad2.a && !lastA) {
            mode = (mode == Mode.ENCODER_ONLY) ? Mode.VISION_TARGET : Mode.ENCODER_ONLY;
            integralSum = 0;
        }
        lastA = gamepad2.a;

        // ================= TERM SELECTION =================
        if (gamepad2.dpad_up && !lastUp) {
            selectedTerm = Term.values()[(selectedTerm.ordinal() + 3) % 4];
        }
        if (gamepad2.dpad_down && !lastDown) {
            selectedTerm = Term.values()[(selectedTerm.ordinal() + 1) % 4];
        }
        lastUp = gamepad2.dpad_up;
        lastDown = gamepad2.dpad_down;

        // ================= VALUE ADJUST =================
        double step =
            selectedTerm == Term.P ? 0.001 :
                selectedTerm == Term.D ? 0.0001 :
                    selectedTerm == Term.I ? 0.0001 : 0.001;

        if (gamepad2.right_bumper && !lastRB) adjustTerm(step);
        if (gamepad2.left_bumper && !lastLB) adjustTerm(-step);
        lastRB = gamepad2.right_bumper;
        lastLB = gamepad2.left_bumper;

        // ================= RESET INTEGRAL =================
        if (gamepad2.x && !lastX) {
            integralSum = 0;
        }
        lastX = gamepad2.x;

        // ================= CURRENT STATE =================
        double currentAngleDeg = turret.getCurrentPosition() * DEG_PER_TICK;

        // ================= TARGET LOGIC =================
        double targetAngleDegTemp;
        double minAngle, maxAngle;

        if (mode == Mode.ENCODER_ONLY) {
            long now = System.currentTimeMillis();
            if (now - lastStepTime > STEP_DURATION_MS) {
                stepIndex = (stepIndex + 1) % testAngles.length;
                lastStepTime = now;
            }
            targetAngleDegTemp = testAngles[stepIndex];

            // Encoder-only range (safe small range for tuning)
            minAngle = -60;
            maxAngle = 60;
        } else { // VISION_TARGET
            targetAngleDegTemp = visionController.getTargetAngle(currentAngleDeg);

            // Vision mode range (full sweep)
            minAngle = -90;
            maxAngle = 90;
        }

        // Clamp target to its mode-specific range
        targetAngleDeg = Math.max(minAngle, Math.min(maxAngle, targetAngleDegTemp));

        // ================= ENCODER PID =================
        double error = targetAngleDeg - currentAngleDeg;
        integralSum += error;
        integralSum = Math.max(-200, Math.min(200, integralSum)); // anti-windup
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integralSum + kD * derivative + kF * Math.signum(error);

        // Optional: clamp power to motor range (can increase for testing)
        power = Math.max(-0.4, Math.min(0.4, power));
        turret.setPower(power);

        // ================= TELEMETRY =================
        telemetry.addLine("=== TURRET PID TEST ===");
        telemetry.addData("Mode", mode);
        telemetry.addData("Selected Term", selectedTerm);

        telemetry.addLine("--- Encoder PID ---");
        telemetry.addData("P", kP);
        telemetry.addData("I", kI);
        telemetry.addData("D", kD);
        telemetry.addData("F", kF);

        telemetry.addLine("--- Angles ---");
        telemetry.addData("Current (deg)", currentAngleDeg);
        telemetry.addData("Target (deg)", targetAngleDeg);
        telemetry.addData("Error (deg)", error);
        telemetry.addData("Min/Max", "[" + minAngle + " / " + maxAngle + "]");
        telemetry.addData("Power", power);
        telemetry.addData("Step Index", stepIndex);
        telemetry.update();
    }


    private void adjustTerm(double delta) {
        switch (selectedTerm) {
            case P: kP += delta; break;
            case I: kI += delta; break;
            case D: kD += delta; break;
            case F: kF += delta; break;
        }
    }
}
