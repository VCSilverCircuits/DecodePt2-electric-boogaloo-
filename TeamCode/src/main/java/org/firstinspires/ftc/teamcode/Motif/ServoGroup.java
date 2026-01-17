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
                      String servo1, String servo2, String servo3, String sensorF1, String sensorB1, String sensorLL) {
        s1 = hw.get(Servo.class, servo1);
        s2 = hw.get(Servo.class, servo2);
        s3 = hw.get(Servo.class, servo3);
    }

    /**
     * Build servo firing order from motif + sensor colors
     */
    public void startMotif(int motifID) {
        // fallback
       /* if (motifID == 21) { // GPP
            sequence = java.util.List.of(servo1, servo2, servo3);
        } else if (motifID == 22) { // PGP
            sequence = java.util.List.of(servo2, servo1, servo3);
        } else if (motifID == 23) { // PPG
            sequence = java.util.List.of(servo3, servo1, servo2);
        } else {
            sequence = java.util.List.of(servo1, servo2, servo3);
        }
        index = 0;
        running = true;
        lastTime = System.currentTimeMillis();
    }

    private void addColor(MatchMotif.MotifPattern motif,
                          ColorSensor1Test sensors,
                          ColorSensor1Test.DetectedColor color) {

        int count = motif == MatchMotif.MotifPattern.GPP && color == ColorSensor1Test.DetectedColor.PURPLE ? 2 :
            motif == MatchMotif.MotifPattern.PGP ? 1 :
                motif == MatchMotif.MotifPattern.PPG && color == ColorSensor1Test.DetectedColor.PURPLE ? 2 : 1;

        for (int i = 0; i < count; i++) {
            if (sensors.getS1() == color) firingOrder.add(s1);
            else if (sensors.getS2() == color) firingOrder.add(s2);
            else if (sensors.getS3() == color) firingOrder.add(s3);
        }
    }


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
    }*/
    }
}
