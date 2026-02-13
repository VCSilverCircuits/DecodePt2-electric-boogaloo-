package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp(name = "ScrimBotTelop")
public class ScrimBotTelop extends OpMode{
    private DcMotorEx Lr;
    private DcMotorEx Rr;
    private DcMotorEx Lf;
    private DcMotorEx Rf;
    private DcMotorEx Launcher;
    private DcMotorEx Intake;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private double TurboMode = 0.5;
    private boolean IfRightTriggerPress = false;
    private double PrevIntakePose = 0;
    private boolean IfLeftTriggerPress = false;
    private double prevflywheelspeed =0;
    private int flyWheelspeed = 1;

    @Override
    public void init() {
        Lr = hardwareMap.get(DcMotorEx.class,"lr");
        Rr = hardwareMap.get(DcMotorEx.class,"rr");
        Lf = hardwareMap.get(DcMotorEx.class,"lf");
        Rf = hardwareMap.get(DcMotorEx.class,"rf");
        Lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lr.setDirection(DcMotorSimple.Direction.FORWARD);
        Rr.setDirection(DcMotorSimple.Direction.REVERSE);
        Lf.setDirection(DcMotorSimple.Direction.FORWARD);
        Rf.setDirection(DcMotorSimple.Direction.REVERSE);
        Launcher = hardwareMap.get(DcMotorEx.class,"Launcher");
        Intake = hardwareMap.get(DcMotorEx.class,"Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class,"flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class,"flywheel2");
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y*TurboMode;
        double strafe = gamepad1.left_stick_x*TurboMode;
        double turn = gamepad1.right_stick_x*0.5;
        Lr.setPower(forward+turn-strafe);
        Rr.setPower(forward-turn+strafe);
        Lf.setPower(forward+turn+strafe);
        Rf.setPower(forward-turn-strafe);
        telemetry.addData("turn",gamepad1.right_stick_x);
        telemetry.update();
        if (gamepad1.left_stick_button){
            TurboMode = 1;
        }else{
            TurboMode = 0.5;
        }
        if (gamepad1.right_bumper){

            Launcher.setPower(0.5);
        }
        else {

            Launcher.setPower(0);
        }
        if (gamepad1.right_trigger>0.5 && !IfRightTriggerPress){
            if (PrevIntakePose>0){
                Intake.setPower(0);
                PrevIntakePose = (0);
            }else {
                Intake.setPower(0.8);
                PrevIntakePose = (0.8);
            }
            IfRightTriggerPress = true;
        }
        if (gamepad1.right_trigger<0.5){
            IfRightTriggerPress = false;
        }
        if (gamepad1.left_trigger>0.5 && !IfLeftTriggerPress){
           if (prevflywheelspeed>0){
               flywheel1.setPower(0);
               flywheel2.setPower(0);
               prevflywheelspeed = (0);
           }else {
               flywheel1.setPower(flyWheelspeed);
               flywheel2.setPower(flyWheelspeed);
               prevflywheelspeed = (1);
           }
           IfLeftTriggerPress = true;
        }
        if (gamepad1.left_trigger<0.5){
            IfLeftTriggerPress = false;
        }
    }
}