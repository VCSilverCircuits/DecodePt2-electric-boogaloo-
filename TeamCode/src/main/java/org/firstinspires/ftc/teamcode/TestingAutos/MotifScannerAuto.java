package org.firstinspires.ftc.teamcode.TestingAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Motif.MotifDetector;
import org.firstinspires.ftc.teamcode.Motif.MatchMotif;

@Autonomous(name = "MotifScannerAuto")
public class MotifScannerAuto extends LinearOpMode {

    private Limelight3A limelight;
    private MotifDetector detector;
    private final ElapsedTime timer = new ElapsedTime();

    // Maximum time to try scanning (in seconds)
    private static final double TIMEOUT = 5.0;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        detector = new MotifDetector(limelight);

        telemetry.addLine("Initializing...");
        telemetry.update();

        // Wait for start
        waitForStart();

        telemetry.addLine("Scanning for motif...");
        telemetry.update();
        timer.reset();

        // Loop until motif detected or timeout
        while (opModeIsActive() && MatchMotif.getPattern() == MatchMotif.MotifPattern.UNKNOWN) {
            detector.update();

            telemetry.addData("Detected Motif", MatchMotif.getPattern());
            telemetry.update();

            if (timer.seconds() > TIMEOUT) {
                telemetry.addLine("Timeout reached. Using UNKNOWN motif.");
                telemetry.update();
                break;
            }
        }

        telemetry.addData("Final Motif", MatchMotif.getPattern());
        telemetry.addLine("Autonomous complete. Ready for TeleOp.");
        telemetry.update();

        sleep(1000); // optional short pause before exit
    }
}
