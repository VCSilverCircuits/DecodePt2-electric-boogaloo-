package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

public class ServoGroup {
    private final Servo servo1, servo2, servo3;
    private final ColorSensor color1, color2, color3;
    private List<Servo> sequence;
    private int index = 0;
    private boolean running = false;
    private long lastTime = 0;
    private final long delayMs = 500; // delay between servos

    public ServoGroup(HardwareMap hardwareMap,
                      String servo1Name, String servo2Name, String servo3Name,
                      String color1Name, String color2Name, String color3Name) {
        servo1 = hardwareMap.get(Servo.class, servo1Name);
        servo2 = hardwareMap.get(Servo.class, servo2Name);
        servo3 = hardwareMap.get(Servo.class, servo3Name);

        color1 = hardwareMap.get(ColorSensor.class, color1Name);
        color2 = hardwareMap.get(ColorSensor.class, color2Name);
        color3 = hardwareMap.get(ColorSensor.class, color3Name);

        // Default sequence order
        sequence = List.of(servo1, servo2, servo3);
    }

    /** Start a sequence based on a motif apriltag */
    public void startMotif(double motifID) {
        // fallback
        if (motifID == 21) { // GPP
            sequence = List.of(servo1, servo2, servo3);
        } else if (motifID == 22) { // PGP
            sequence = List.of(servo2, servo1, servo3);
        } else if (motifID == 23) { // PPG
            sequence = List.of(servo3, servo1, servo2);
        } else {
            sequence = List.of(servo1, servo2, servo3);
        }
        index = 0;
        running = true;
        lastTime = System.currentTimeMillis();
    }

    /** Non-blocking loop to process the sequence */
    public void loop() {
        if (!running) return;
        long now = System.currentTimeMillis();
        if (index >= sequence.size()) {
            running = false;
            return;
        }
        if (now - lastTime >= delayMs) {
            Servo s = sequence.get(index);
            s.setPosition(1); // flick servo up
            lastTime = now;
            index++;
        }
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        running = false;
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }
}
