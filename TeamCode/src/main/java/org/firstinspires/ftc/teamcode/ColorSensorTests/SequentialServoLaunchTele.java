package org.firstinspires.ftc.teamcode.ColorSensorTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Ordered Multi-Servo Launcher")
public class OrderedMultiServoLauncherTeleOp extends OpMode {

    private ColorSensor1Test bench = new ColorSensor1Test();

    private Servo servo1, servo2, servo3;

    // Action control
    private boolean actionLatched = false;
    private int currentStep = 0; // 0 = idle, 1..n = servo step
    private ElapsedTime actionTimer = new ElapsedTime();
    private boolean lastA = false;

    // Launch order array (indices: 0 = servo1, 1 = servo2, 2 = servo3)
    private int[] launchOrder = {0,1,2}; // default order

    @Override
    public void init() {
        bench.init(hardwareMap);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        // safe default
        servo1.setPosition(0.0);
        servo2.setPosition(0.0);
        servo3.setPosition(0.0);

        // Set the launch order based on motif pattern
        setLaunchOrderForMotif();
    }

    private void setLaunchOrderForMotif() {
        // Example: could use MatchMotif class or your own logic
        // 0 = servo1, 1 = servo2, 2 = servo3
        // GPP -> fire in order 1,2,3 for example
        // PGP -> 2,1,3
        // PPG -> 2,3,1
        ColorSensor1Test.DetectedColor[] motif = new ColorSensor1Test.DetectedColor[3];
        // This is a placeholder: replace with your motif detection
        motif[0] = bench.getS1();
        motif[1] = bench.getS2();
        motif[2] = bench.getDetectedColorS3();

        // Determine order based on color pattern
        // Example logic (customize to your mapping)
        launchOrder = new int[]{0,1,2}; // default
        if (motif[0] == ColorSensor1Test.DetectedColor.GREEN &&
            motif[1] == ColorSensor1Test.DetectedColor.PURPLE &&
            motif[2] == ColorSensor1Test.DetectedColor.PURPLE) {
            launchOrder = new int[]{0,1,2}; // GPP
        } else if (motif[0] == ColorSensor1Test.DetectedColor.PURPLE &&
            motif[1] == ColorSensor1Test.DetectedColor.GREEN &&
            motif[2] == ColorSensor1Test.DetectedColor.PURPLE) {
            launchOrder = new int[]{1,0,2}; // PGP
        } else if (motif[0] == ColorSensor1Test.DetectedColor.PURPLE &&
            motif[1] == ColorSensor1Test.DetectedColor.PURPLE &&
            motif[2] == ColorSensor1Test.DetectedColor.GREEN) {
            launchOrder = new int[]{1,2,0}; // PPG
        }
    }

    private Servo getServoByIndex(int i) {
        switch (i) {
            case 0: return servo1;
            case 1: return servo2;
            case 2: return servo3;
            default: return null;
        }
    }

    @Override
    public void loop() {

        bench.update(); // update sensors

        boolean aPressed = gamepad1.a;

        // =========================
        // START SEQUENCE (edge-triggered)
        // =========================
        if (!actionLatched && bench.allSensorsHaveColor() && aPressed && !lastA) {
            actionLatched = true;
            currentStep = 0;   // index in launchOrder
            actionTimer.reset();
        }

        // =========================
        // RUN SEQUENCE
        // =========================
        if (actionLatched) {
            Servo currentServo = getServoByIndex(launchOrder[currentStep]);
            currentServo.setPosition(1.0); // launch

            if (actionTimer.seconds() > 0.5) { // hold time
                currentServo.setPosition(0.0); // reset
                currentStep++;

                if (currentStep >= launchOrder.length) {
                    // sequence complete
                    actionLatched = false;
                    currentStep = 0;
                } else {
                    actionTimer.reset();
                }
            }
        }

        // =========================
        // MANUAL RESET
        // =========================
        if (gamepad1.b) {
            bench.reset();
            actionLatched = false;
            currentStep = 0;
            servo1.setPosition(0.0);
            servo2.setPosition(0.0);
            servo3.setPosition(0.0);
        }

        // =========================
        // TELEMETRY
        // =========================
        telemetry.addData("Action Latched", actionLatched);
        telemetry.addData("Current Step", currentStep);
        telemetry.addData("Launch Order", launchOrder[0]+","+launchOrder[1]+","+launchOrder[2]);
        telemetry.addData("Servo1", servo1.getPosition());
        telemetry.addData("Servo2", servo2.getPosition());
        telemetry.addData("Servo3", servo3.getPosition());
        telemetry.update();

        lastA = aPressed;
    }
}

}
