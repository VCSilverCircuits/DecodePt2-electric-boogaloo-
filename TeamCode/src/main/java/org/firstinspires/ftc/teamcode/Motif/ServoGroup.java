package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensor1Test;

import java.util.ArrayList;
import java.util.List;

public class ServoGroup {

    private final Servo s1, s2, s3;
    private final List<Servo> firingOrder = new ArrayList<>();

    private int index = 0;
    private boolean running = false;
    private boolean servoUp = false;
    private long lastTime = 0;

    private static final long DELAY_MS = 400;

    public ServoGroup(HardwareMap hw,
                      String servo1, String servo2, String servo3) {
        s1 = hw.get(Servo.class, servo1);
        s2 = hw.get(Servo.class, servo2);
        s3 = hw.get(Servo.class, servo3);
    }

    /**
     * Build servo firing order from motif + latched sensor colors
     */
    public void startMotif(MatchMotif.MotifPattern motif, ColorSensor1Test sensors) {

        firingOrder.clear();

        // Map sensors to servos
        ColorSensor1Test.DetectedColor[] sensorColors = new ColorSensor1Test.DetectedColor[]{
            sensors.getS1(), // s1
            sensors.getS2(), // s2
            sensors.getS3()  // s3
        };

        Servo[] servos = new Servo[]{s1, s2, s3};

        // Build the sequence according to motif
        ColorSensor1Test.DetectedColor[] motifColors = new ColorSensor1Test.DetectedColor[3];

        switch (motif) {
            case GPP: // Green, Purple, Purple
                motifColors[0] = ColorSensor1Test.DetectedColor.GREEN;
                motifColors[1] = ColorSensor1Test.DetectedColor.PURPLE;
                motifColors[2] = ColorSensor1Test.DetectedColor.PURPLE;
                break;
            case PGP: // Purple, Green, Purple
                motifColors[0] = ColorSensor1Test.DetectedColor.PURPLE;
                motifColors[1] = ColorSensor1Test.DetectedColor.GREEN;
                motifColors[2] = ColorSensor1Test.DetectedColor.PURPLE;
                break;
            case PPG: // Purple, Purple, Green
                motifColors[0] = ColorSensor1Test.DetectedColor.PURPLE;
                motifColors[1] = ColorSensor1Test.DetectedColor.PURPLE;
                motifColors[2] = ColorSensor1Test.DetectedColor.GREEN;
                break;
            default:
                // fallback: just fire in sensor order
                firingOrder.add(s1);
                firingOrder.add(s2);
                firingOrder.add(s3);
                this.index = 0;
                this.running = true;
                this.lastTime = System.currentTimeMillis();
                return;
        }

        // For each color in motif, find the servo whose sensor matches it
        for (ColorSensor1Test.DetectedColor c : motifColors) {
            for (int i = 0; i < sensorColors.length; i++) {
                if (sensorColors[i] == c) {
                    firingOrder.add(servos[i]);
                    break; // move to next motif color
                }
            }
        }

        index = 0;
        running = true;
        lastTime = System.currentTimeMillis();
    }

    /**
     * Call repeatedly in loop to fire servos
     */
    public void loop() {
        if (!running || index >= firingOrder.size()) return;

        long now = System.currentTimeMillis();
        if (now - lastTime < DELAY_MS) return;

        Servo current = firingOrder.get(index);

        if (!servoUp) {
            current.setPosition(1);
            servoUp = true;
        } else {
            current.setPosition(0);
            servoUp = false;
            index++;
        }

        lastTime = now;
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        running = false;
        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0);
    }
}
