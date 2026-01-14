package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor1Test {
    NormalizedColorSensor sensor1;

    public enum detectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }
    public void init(HardwareMap hwMap){
        sensor1 = hwMap.get(NormalizedColorSensor.class, "sensor1");

    }
    public detectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = sensor1.getNormalizedColors();
        float normaRed, normaGreen,normaBlue;
        normaRed = colors.red / colors.alpha;
        normaGreen = colors.green / colors.alpha;
        normaBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normaRed);
        telemetry.addData("green", normaGreen);
        telemetry.addData("blue",normaBlue);
        return detectedColor.UNKNOWN;
    }
}
