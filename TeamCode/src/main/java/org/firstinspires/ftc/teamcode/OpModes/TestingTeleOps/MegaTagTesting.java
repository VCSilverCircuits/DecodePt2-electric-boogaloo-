package org.firstinspires.ftc.teamcode.OpModes.TestingTeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.OdoAim;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp(name="MegaTag")
public class MegaTagTesting extends OpMode {
    Limelight3A limelight;
    LLResult result;
    OdoAim turret;
    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        turret = new OdoAim(hardwareMap, follower, true);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
        turret.idle();
    }

    @Override
    public void loop() {

        follower.update();

        telemetry.addData("Result Null?", result == null);
        telemetry.addData("Result Valid?", result != null && result.isValid());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            Pose3D botpose = result.getBotpose();
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
            double heading = Math.toRadians(botpose.getOrientation().getYaw());

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa();
            if (botpose != null) {

                Pose tagPose = new Pose(x, y, heading);

                Pose currentPose = follower.getPose();

                double error = currentPose.distanceFrom(tagPose);

                if (error > 1.0) {
                    follower.setPose(tagPose);
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                }

                telemetry.addData("Tag Pose", tagPose);
                telemetry.addData("Pose Error", error);
            }

            telemetry.addData("Follower Pose", follower.getPose());
            telemetry.update();
        }
    }
}
