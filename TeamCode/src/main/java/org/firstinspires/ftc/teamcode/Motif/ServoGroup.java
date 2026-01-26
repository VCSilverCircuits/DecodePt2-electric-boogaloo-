package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensors;

import java.util.ArrayList;
import java.util.List;

public class ServoGroup {

    private final Servo s1, s2, s3;
    private final List<Servo> firingOrder = new ArrayList<>();

    private int index = 0;           // current servo in sequence
    private boolean servoUp = false; // is the current servo up?
    private long lastTime = 0;       // last time we moved a servo
    private boolean running = false;

    // Timing constants
    private static final long UP_DURATION_MS = 300;   // how long servo stays up
    private static final long DOWN_DELAY_MS = 400;    // wait after servo goes down before next

    public ServoGroup(HardwareMap hw,
                      String servo1, String servo2, String servo3) {
        s1 = hw.get(Servo.class, servo1);
        s2 = hw.get(Servo.class, servo2);
        s3 = hw.get(Servo.class, servo3);

        // Make sure all servos start down
        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0);
    }

    /**
     * Build servo firing order from motif + sensor colors
     */
    public void startMotif(MatchMotif.MotifPattern motif, ColorSensors sensors) {
        firingOrder.clear();
        index = 0;
        running = true;
        servoUp = false;
        lastTime = System.currentTimeMillis();

        // Add servos based on motif pattern and detected colors
        addColor(motif, sensors, ColorSensors.DetectedColor.GREEN);
        addColor(motif, sensors, ColorSensors.DetectedColor.PURPLE);
    }

    /**
     * Add servos corresponding to a color in the motif
     */
    private void addColor(MatchMotif.MotifPattern motif,
                          ColorSensors sensors,
                          ColorSensors.DetectedColor color) {

        int count = 1;
        switch (motif) {
            case GPP:
                if (color == ColorSensors.DetectedColor.PURPLE) count = 2;
                break;
            case PGP:
                count = 1; // one green and two purples handled by loop
                break;
            case PPG:
                if (color == ColorSensors.DetectedColor.PURPLE) count = 2;
                break;
            default:
                count = 1;
        }

        for (int i = 0; i < count; i++) {
            if (sensors.getS1() == color) firingOrder.add(s1);
            else if (sensors.getS2() == color) firingOrder.add(s2);
            else if (sensors.getS3() == color) firingOrder.add(s3);
        }
    }

    /**
     * Call this repeatedly in loop() to handle firing sequence
     */
    public void loop() {
        if (!running || index >= firingOrder.size()) return;

        long now = System.currentTimeMillis();

        Servo current = firingOrder.get(index);

        if (servoUp) {
            // Servo is currently up
            if (now - lastTime >= UP_DURATION_MS) {
                current.setPosition(0); // move down
                servoUp = false;
                lastTime = now;
                index++; // move to next servo after down
            }
        } else {
            // Servo is currently down
            if (now - lastTime >= DOWN_DELAY_MS) {
                if (index < firingOrder.size()) {
                    current.setPosition(1); // move up
                    servoUp = true;
                    lastTime = now;
                }
            }
        }
    }

    public boolean isRunning() {
        return running && index < firingOrder.size();
    }

    public void stop() {
        running = false;
        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0);
        firingOrder.clear();
        index = 0;
        servoUp = false;
    }
}
