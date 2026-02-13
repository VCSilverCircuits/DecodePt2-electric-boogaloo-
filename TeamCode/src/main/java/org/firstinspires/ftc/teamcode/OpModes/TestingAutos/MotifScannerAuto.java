package org.firstinspires.ftc.teamcode.OpModes.TestingAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Motif.MotifDetector;
import org.firstinspires.ftc.teamcode.Subsystems.Motif.MatchMotif;

@Autonomous(name = "MotifScannerAuto")
public class MotifScannerAuto extends LinearOpMode {

    private MotifDetector detector;
    private final ElapsedTime timer = new ElapsedTime();

    // Maximum time to try scanning (in seconds)
    private static final double TIMEOUT = 5.0;

    @Override
    public void runOpMode() {

        // 🔥 HARD RESET — must happen before anything else
        MatchMotif.reset();

        // Init detector AFTER reset
        detector = new MotifDetector(hardwareMap);

        telemetry.addLine("Initializing...");
        telemetry.addData("Motif after reset", MatchMotif.getPattern());
        telemetry.update();

        // Wait for start
        waitForStart();

        telemetry.addLine("Scanning for motif...");
        telemetry.update();

        timer.reset();

        // Loop until motif detected or timeout
        while (opModeIsActive()
            && MatchMotif.getPattern() == MatchMotif.MotifPattern.UNKNOWN) {

            detector.update();

            telemetry.addData("Detected Motif", MatchMotif.getPattern());
            telemetry.addData("Time", timer.seconds());
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

        sleep(1000); // optional pause before exit
    }
}
