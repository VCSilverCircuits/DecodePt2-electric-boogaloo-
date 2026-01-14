package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "Beta Tele")
public class BetaTele extends OpMode {
    private Follower follower;


    // Turret
    private DcMotorEx turret;
    //private AprilTagTurretController turretController;

    private DcMotorEx intake;
    private boolean intakeToggle = false;
    private boolean lastRightTriggerPressed = false;

    private boolean turretTrackingEnabled = true;

    // --- encoder → degrees conversion ---
    private static final double BELT_RATIO = 230.0 / 20.0; // motor : turret
    private static final double TICKS_PER_MOTOR_REV = 537.6; // example: NeveRest 20
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_MOTOR_REV / BELT_RATIO;

    @Override
    public void init() {
        // Turret
        turret = hardwareMap.get(DcMotorEx.class, "turretRotation");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // AprilTag turret controller
        //turretController = new AprilTagTurretController(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.54, 125.169, 143));
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        // --- Intake toggle ---
        boolean rtPressed = gamepad1.right_trigger > 0.5;
        if (rtPressed && !lastRightTriggerPressed) intakeToggle = !intakeToggle;
        lastRightTriggerPressed = rtPressed;
        intake.setPower(intakeToggle ? -1 : 0);
        //driving
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x * 0.5,
            true
        );
        // --- Turret control ---
        // Toggle tracking
        if (gamepad1.a) turretTrackingEnabled = true;
        if (gamepad1.b) {
            turretTrackingEnabled = false;
            turret.setPower(0);
        }

        // Reverse search direction manually
        //if (gamepad1.x) turretController.reverseSearchDirection();

        // Only compute and set turret power if tracking enabled
        /*if (turretTrackingEnabled) {
            double currentTurretAngleDeg = turret.getCurrentPosition() * DEGREES_PER_TICK;
            double turretPower = turretController.getTurretPower(currentTurretAngleDeg);
            turret.setPower(turretPower);
        }

            telemetry.addData("Turret Tracking", turretTrackingEnabled);
            telemetry.addData("Current Angle", turret.getCurrentPosition() * DEGREES_PER_TICK);
            telemetry.addData("Has Lock", turretController.isLocked());
            telemetry.addData("Last Seen Angle", turretController.getLastSeenAngle());
            telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void stop() {
        MatchMotif.reset();
        turret.setPower(0);
        intake.setPower(0);
    }
    */
    }
}


