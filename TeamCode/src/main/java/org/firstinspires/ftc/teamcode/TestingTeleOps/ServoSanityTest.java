package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Sanity Test")
public class ServoSanityTest extends OpMode {

    private Servo servo1;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "frontFlipper");
        servo1.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo1.setPosition(0);
        } else {
            servo1.setPosition(0.5);
        }

        telemetry.addData("Servo Pos", servo1.getPosition());
        telemetry.update();
    }
}
