package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;


// PEDRO PATHING IMPORTS
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp(name = "Ursa TeleOp")
public class UrsaTestTele extends OpMode {

    // Subsystems
    private OdoAim turret;

    // Pedro Pathing Follower (Handles Drivetrain)
    private Follower follower;

    // Telemetry Manager
    private TelemetryManager telemetryManager;

    @Override
    public void init() {
        /// Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // Optimize Hardware Reads
        turret = new OdoAim(hardwareMap,follower,true);

        // Initialize Telemetry Manager
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {follower.startTeleOpDrive();
        follower.setStartingPose(new Pose(84.71028037383178, 39.813084112149525, Math.toRadians(0))); }// example start}

    @Override
    public void loop() {
        // --- 1. DRIVETRAIN (PedroPathing) ---
        // Stick Y is inverted (Up is negative on standard gamepads)
        turret.update();
        turret.odoAim();
        follower.setTeleOpDrive(
            -gamepad1.left_stick_x, // Forward/Back
            +gamepad1.left_stick_y, // Strafe
            -gamepad1.right_stick_x * 0.6, // Turn
            true // TRUE = Robot Centric
        );
        follower.update();

        // --- Reset Yaw Encoder ---
        if (gamepad1.b) {
            turret.idle();
        }


        // Telemetry
        telemetry.addLine("Use Dpad left and right to adjust gate position");
        telemetry.addData("Flywheel Target", "See Dashboard");
        telemetry.addData("turretPosition", turret.getTurretPosition());
        telemetry.addData("turretPosition", turret.getRelativeTargetHeading());
        telemetry.update(); telemetryManager.update();
    }
}
