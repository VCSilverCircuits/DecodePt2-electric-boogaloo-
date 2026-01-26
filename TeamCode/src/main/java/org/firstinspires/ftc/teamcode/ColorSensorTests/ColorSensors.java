package org.firstinspires.ftc.teamcode.ColorSensorTests;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensors {

    private NormalizedColorSensor sensor1, sensor2, sensor3;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    private DetectedColor latched1 = DetectedColor.UNKNOWN;
    private DetectedColor latched2 = DetectedColor.UNKNOWN;
    private DetectedColor latched3 = DetectedColor.UNKNOWN;

    public void init(HardwareMap hwMap) {
        sensor1 = hwMap.get(NormalizedColorSensor.class, "sensorF1");
        sensor2 = hwMap.get(NormalizedColorSensor.class, "sensorB1");
        sensor3 = hwMap.get(NormalizedColorSensor.class, "sensorLL");

        sensor1.setGain(10);
        sensor2.setGain(10);
        sensor3.setGain(10);
    }

    public void update() {
        if (latched1 == DetectedColor.UNKNOWN) latched1 = detect(sensor1, 1);
        if (latched2 == DetectedColor.UNKNOWN) latched2 = detect(sensor2, 2);
        if (latched3 == DetectedColor.UNKNOWN) latched3 = detect(sensor3, 3);
    }

    private DetectedColor detect(NormalizedColorSensor sensor, int index) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        if (c.alpha == 0) return DetectedColor.UNKNOWN;

        float r = c.red / c.alpha;
        float g = c.green / c.alpha;
        float b = c.blue / c.alpha;

        switch (index) {
            case 1:
                if (r < 0.34 && g < 0.401 && b > 0.6) return DetectedColor.PURPLE;
                if (r < 0.14 && g > 0.30 && b < 0.52) return DetectedColor.GREEN;
                break;
            case 2:
                if (r < 0.46 && g < 0.43 && b > 0.29) return DetectedColor.PURPLE;
                if (r < 0.14 && g > 0.30 && b < 0.52) return DetectedColor.GREEN;
                break;
            case 3:
                if (r < 0.47 && g < 0.51 && b > 0.3) return DetectedColor.PURPLE;
                if (r < 0.21 && g > 0.2 && b < 0.6) return DetectedColor.GREEN;
                break;
        }
        return DetectedColor.UNKNOWN;
    }

    public DetectedColor getS1() { return latched1; }
    public DetectedColor getS2() { return latched2; }
    public DetectedColor getS3() { return latched3; }

    public boolean allSensorsHaveColor() {
        return latched1 != DetectedColor.UNKNOWN &&
            latched2 != DetectedColor.UNKNOWN &&
            latched3 != DetectedColor.UNKNOWN;
    }

    public void reset() {
        latched1 = latched2 = latched3 = DetectedColor.UNKNOWN;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Sensor 1", latched1);
        telemetry.addData("Sensor 2", latched2);
        telemetry.addData("Sensor 3", latched3);
    }
}
