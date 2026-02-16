package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Hood Servo Adjuster", group = "TeleOp")
public class HoodServoAdjuster extends OpMode {
    private FlywheelConstants flywheel;
    private Follower follower;
    private OdoAim turret;

    // Servo for hood
    private Servo hoodServo;

    // Adjustable hood angle in degrees
    private double hoodAngle = 20; // starting angle
    private final double minAngle = 10;
    private final double maxAngle = 40;
    private final double step = 1; // degrees per button press

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = new OdoAim(hardwareMap, follower, true);
        flywheel = new FlywheelConstants(hardwareMap, follower);
        hoodServo = hardwareMap.get(Servo.class, "Hood"); // name of your hood servo
        setServoAngle(hoodAngle);

        telemetry.addLine("Hood Servo Adjuster Initialized");
        telemetry.addData("Hood Angle", hoodAngle);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(122.0187, 123.8131, Math.toRadians(37))); // example start
        flywheel.enable();
    }

    @Override
    public void loop() {
        flywheel.update(-gamepad1.left_stick_y * 50);
        follower.update();
        turret.update();
        turret.odoAim();

        // Adjust hood angle with gamepad buttons
        if (gamepad1.x) {
            hoodAngle += step;
        }
        if (gamepad1.b) {
            hoodAngle -= step;
        }

        // Clamp to safe servo range
        hoodAngle = Math.max(minAngle, Math.min(maxAngle, hoodAngle));

        // Apply to servo
        setServoAngle(hoodAngle);

        // Telemetry
        telemetry.addData("Hood Angle (deg)", hoodAngle);
        telemetry.update();
    }

    private void setServoAngle(double angle) {
        // Map hood angle to servo position (0-1)
        double position = (angle - minAngle) / (maxAngle - minAngle);
        position = Math.max(0, Math.min(1, position));
        hoodServo.setPosition(position);
    }

    @Override
    public void stop() {
        // optional: leave servo at last position
    }
}
