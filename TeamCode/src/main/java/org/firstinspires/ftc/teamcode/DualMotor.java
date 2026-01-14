package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DualMotor {

    private DcMotorEx motor1, motor2;

    public DualMotor(DcMotor motor1, DcMotor motor2) {
        this.motor1 = (DcMotorEx) motor1;
        this.motor2 = (DcMotorEx) motor2;
    }

    // Set power (0 to 1)
    public void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    // Set velocity in ticks/second or RPM (your choice)
    public void setVelocity(double ticksPerSecond) {
        motor1.setVelocity(ticksPerSecond);
        motor2.setVelocity(ticksPerSecond);
    }

    // Converts RPM → ticks/sec for you
    public void setVelocityRPM(double rpm, double ticksPerRev) {
        double tps = (rpm * ticksPerRev) / 60.0;
        setVelocity(tps);
    }

    // Set directions independently
    public void setDirections(DcMotorSimple.Direction dir1, DcMotorSimple.Direction dir2) {
        motor1.setDirection(dir1);
        motor2.setDirection(dir2);
    }

    // Stop both motors
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
    }

    // Set BRAKE or FLOAT
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor1.setZeroPowerBehavior(behavior);
        motor2.setZeroPowerBehavior(behavior);
    }
}


