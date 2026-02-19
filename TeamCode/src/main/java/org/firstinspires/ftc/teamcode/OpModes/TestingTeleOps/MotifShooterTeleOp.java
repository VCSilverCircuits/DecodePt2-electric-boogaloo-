package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.MatchMotif;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.ServoGroup;

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
            "frontFlipper", "backFlipper", "leftFlipper", "stopper"
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
                telemetry.addData("Firing servo index", servos.getIndex());
            } else if (isFiring) {
                // Sequence finished, reset for next intake
                servos.stop();
                sensors.reset(); // optional: clear previous colors so next update re-latches
                isFiring = false;
                telemetry.addLine("Sequence complete. Ready to fire again.");
            }
            telemetry.addData("ServoGroup running", servos.isRunning());
            telemetry.addData("Current index", servos.getIndex());
            telemetry.addData("Sequence length", servos.getSequenceLength());
            telemetry.addData("Motif", MatchMotif.getPattern());
            telemetry.addData("S1 color", sensors.getS1());
            telemetry.addData("S2 color", sensors.getS2());
            telemetry.addData("S3 color", sensors.getS3());
            telemetry.update();
        }

        // Ensure everything is down at the end
        servos.stop();
    }

    // Helper to show which servo is currently firing
}
