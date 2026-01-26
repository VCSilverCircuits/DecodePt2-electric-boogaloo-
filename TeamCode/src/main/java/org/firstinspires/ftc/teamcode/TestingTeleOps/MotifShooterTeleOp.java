package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Motif.MatchMotif;
import org.firstinspires.ftc.teamcode.Motif.ServoGroup;

@TeleOp(name = "Motif TeleOp")
public class MotifShooterTeleOp extends LinearOpMode {

    private ColorSensors sensors;
    private ServoGroup servos;
    private boolean isFiring = false; // Tracks if sequence is running

    @Override
    public void runOpMode() {

        // Initialize sensors and servos
        sensors = new ColorSensors();
        sensors.init(hardwareMap);

        servos = new ServoGroup(
            hardwareMap,
            "servo1", "servo2", "servo3"
        );

        telemetry.addLine("Ready. Press 'A' to fire sequence.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Fire sequence when 'A' pressed and not already firing
            if (gamepad1.a && !isFiring) {
                // Latch current colors
                sensors.reset(); // clear previous latches
                sensors.update();

                // Start servo sequence based on motif + current sensor colors
                servos.startMotif(MatchMotif.getPattern(), sensors);

                isFiring = true;
            }

            // Run the servo sequence if active
            if (servos.isRunning()) {
                servos.loop();
                telemetry.addData("Firing servo index", getServoIndex());
            } else if (isFiring) {
                // Sequence finished, reset for next intake
                servos.stop();
                sensors.reset(); // optional: clear previous colors so next update re-latches
                isFiring = false;
                telemetry.addLine("Sequence complete. Ready to fire again.");
            }

            telemetry.update();
        }

        // Ensure everything is down at the end
        servos.stop();
    }

    // Helper to show which servo is currently firing
    private int getServoIndex() {
        try {
            java.lang.reflect.Field field = ServoGroup.class.getDeclaredField("index");
            field.setAccessible(true);
            return field.getInt(servos);
        } catch (Exception e) {
            return -1;
        }
    }
}
