package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ColorSensorTests.ColorSensors;

import java.util.ArrayList;
import java.util.List;

public class ServoGroup {

    private final Servo s1, s2, s3;
    private final List<Servo> firingOrder = new ArrayList<>();

    private int index = 0;
    private boolean servoUp = false;
    private long lastTime = 0;
    private boolean running = false;

    // Timing constants
    private static final long UP_DURATION_MS = 500;
    private static final long DOWN_DELAY_MS = 500;

    public ServoGroup(HardwareMap hw,
                      String servo1, String servo2, String servo3) {

        s1 = hw.get(Servo.class, servo1);
        s2 = hw.get(Servo.class, servo2);
        s3 = hw.get(Servo.class, servo3);

        stop(); // ensure known state
    }

    /**
     * Start firing sequence based on motif + latched sensor colors
     */
    public void startMotif(MatchMotif.MotifPattern motif, ColorSensors sensors) {

        firingOrder.clear();
        index = 0;
        servoUp = false;
        running = true;
        lastTime = System.currentTimeMillis();

        // Build firing order explicitly
        buildFiringOrder(motif, sensors);

        // 🚑 Absolute safety: never allow empty sequence
        if (firingOrder.isEmpty()) {
            firingOrder.add(s1);
            firingOrder.add(s2);
            firingOrder.add(s3);
        }
    }

    /**
     * Builds firing order in exact motif order
     */
    private void buildFiringOrder(MatchMotif.MotifPattern motif, ColorSensors sensors) {
        // Build exact color sequence for this motif
        ColorSensors.DetectedColor[] pattern;

        switch (motif) {
            case GPP:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
            case PGP:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
            case PPG:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.GREEN
                };
                break;
            default: // UNKNOWN
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
        }

        // Add servos to firing order according to pattern
        for (ColorSensors.DetectedColor color : pattern) {
            if (sensors.getS1() == color) firingOrder.add(s1);
            else if (sensors.getS2() == color) firingOrder.add(s2);
            else if (sensors.getS3() == color) firingOrder.add(s3);
        }
    }


    /**
     * Call repeatedly in TeleOp loop
     */
    public void loop() {
        if (!running || index >= firingOrder.size()) return;

        long now = System.currentTimeMillis();
        Servo current = firingOrder.get(index);

        if (servoUp) {
            if (now - lastTime >= UP_DURATION_MS) {
                current.setPosition(0);
                servoUp = false;
                lastTime = now;
                index++;
            }
        } else {
            if (now - lastTime >= DOWN_DELAY_MS) {
                current.setPosition(1);
                servoUp = true;
                lastTime = now;
            }
        }
    }

    public boolean isRunning() {
        return running && index < firingOrder.size();
    }

    public int getIndex() {
        return index;
    }

    public int getSequenceLength() {
        return firingOrder.size();
    }

    public void stop() {
        running = false;
        index = 0;
        servoUp = false;
        firingOrder.clear();

        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0);
    }
}
