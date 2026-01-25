package org.firstinspires.ftc.teamcode.ColorSensorTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Sequential Servo Launch with Sensor Lock")
public class SequentialServoLaunchTele extends OpMode {

    private ColorSensor1Test bench = new ColorSensor1Test();

    private Servo servo1, servo2, servo3;
    private Servo[] servos;
    private int currentServoIndex = 0;

    private boolean sequenceRunning = false;
    private ElapsedTime timer = new ElapsedTime();

    private static final double LAUNCH_POSITION = 1.0; // up
    private static final double REST_POSITION   = 0; // down
    private static final double HOLD_TIME_SEC   = 0.5; // hold at up position

    private boolean lastA = false;

    @Override
    public void init() {
        bench.init(hardwareMap);

        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo2 = hardwareMap.get(Servo.class, "backFlipper");
        servo3 = hardwareMap.get(Servo.class, "leftFlipper");

        servos = new Servo[]{servo1, servo2, servo3};

        // Make sure all start at rest
        servo1.setPosition(0.0);
        servo2.setPosition(0.0);
        servo3.setPosition(0.0);

    }

    @Override
    public void loop() {
        bench.update(); // make sure sensors refresh

        boolean aPressed = gamepad1.a;

        // Start the sequence if button pressed AND all sensors detect a color
        if (aPressed && !lastA && !sequenceRunning && bench.allSensorsHaveColor()) {
            sequenceRunning = true;
            currentServoIndex = 0;
            timer.reset();
            servos[currentServoIndex].setPosition(LAUNCH_POSITION);
        }
        // Sequence running logic
        if (sequenceRunning) {
            if (timer.seconds() > HOLD_TIME_SEC) {
                // Return current servo to rest
                servos[currentServoIndex].setPosition(REST_POSITION);
                // Move to next servo
                currentServoIndex++;

                if (currentServoIndex >= servos.length) {
                    // Finished all servos
                    sequenceRunning = false;
                } else {
                    // Launch next servo
                    servos[currentServoIndex].setPosition(LAUNCH_POSITION);
                    timer.reset();
                }
            }
        }

        lastA = aPressed;

        // Optional: manual reset
        if (gamepad1.b) {
            sequenceRunning = false;
            currentServoIndex = 0;
            for (Servo s : servos) s.setPosition(REST_POSITION);
        }

        // Telemetry
        telemetry.addData("Sequence Running", sequenceRunning);
        telemetry.addData("Current Servo", currentServoIndex + 1);
        telemetry.addData("All Sensors Valid", bench.allSensorsHaveColor());
        for (int i = 0; i < servos.length; i++) {
            telemetry.addData("Servo " + (i + 1), servos[i].getPosition());
        }
        telemetry.update();
    }
}
