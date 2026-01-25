package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensor1Test;
import org.firstinspires.ftc.teamcode.Motif.MatchMotif;
import org.firstinspires.ftc.teamcode.Motif.ServoGroup;

@TeleOp(name = "MotifShooterButtonTeleOp")
public class MotifShooterTeleOp extends LinearOpMode {

    private ColorSensor1Test sensors;
    private ServoGroup servos;
    private boolean hasFired = false; // Prevent multiple triggers per button press

    @Override
    public void runOpMode() {

        // Initialize color sensors
        sensors = new ColorSensor1Test();
        sensors.init(hardwareMap);

        // Initialize servos
        servos = new ServoGroup(
            hardwareMap,
            "servo1", "servo2", "servo3"
        );

        telemetry.addLine("Ready. Press 'A' to fire the motif sequence.");
        telemetry.update();

        waitForStart();

        // Latch colors from sensors once at start
        sensors.update();

        telemetry.addLine("Latched Colors:");
        telemetry.addData("Sensor F1 (s1)", sensors.getS1());
        telemetry.addData("Sensor B1 (s2)", sensors.getS2());
        telemetry.addData("Sensor LL (s3)", sensors.getS3());
        telemetry.update();

        // Get motif detected in Auto
        MatchMotif.MotifPattern pattern = MatchMotif.getPattern();
        telemetry.addData("Motif from Auto", pattern);
        telemetry.update();

        // Main loop
        while (opModeIsActive()) {

            // Trigger firing when 'A' is pressed and hasn't fired yet
            if (gamepad1.a && !hasFired) {
                servos.startMotif(pattern, sensors);
                hasFired = true; // Only trigger once per press
            }

            // Reset trigger if button released
            if (!gamepad1.a && hasFired && !servos.isRunning()) {
                hasFired = false;
            }

            // Run servo sequence if active
            if (servos.isRunning()) {
                servos.loop();
                telemetry.addData("Firing servo index", servosIndex());
            }

            telemetry.update();
        }

        // Make sure all servos are down at the end
        servos.stop();
    }

    // Helper to show which servo is currently firing
    private int servosIndex() {
        try {
            java.lang.reflect.Field field = ServoGroup.class.getDeclaredField("index");
            field.setAccessible(true);
            return field.getInt(servos);
        } catch (Exception e) {
            return -1;
        }
    }
}
