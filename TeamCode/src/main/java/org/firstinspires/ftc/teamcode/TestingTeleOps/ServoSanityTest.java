package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Sanity Test")
public class ServoSanityTest extends OpMode {

    private Servo servo3;

    @Override
    public void init() {
        servo3 = hardwareMap.get(Servo.class, "backFlipper");
        servo3.setPosition(0.2);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo3.setPosition(0.78);
        } else {
            servo3.setPosition(0.2);
        }

        telemetry.addData("Servo Pos", servo3.getPosition());
        telemetry.update();
    }
}
