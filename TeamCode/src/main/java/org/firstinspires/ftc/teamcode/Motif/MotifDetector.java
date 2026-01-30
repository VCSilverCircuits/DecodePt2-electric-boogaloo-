package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class MotifDetector {

    private final Limelight3A limelight;
    private boolean detected = false;

    private static final int GPP_ID = 21;
    private static final int PGP_ID = 22;
    private static final int PPG_ID = 23;

    public MotifDetector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
    public void reset() {
        detected = false;
    }


    public void update() {
        if (detected) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {
            switch (tag.getFiducialId()) {
                case GPP_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.GPP);
                    detected = true;
                    return;
                case PGP_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.PGP);
                    detected = true;
                    return;
                case PPG_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.PPG);
                    detected = true;
                    return;
            }
        }
    }
}
