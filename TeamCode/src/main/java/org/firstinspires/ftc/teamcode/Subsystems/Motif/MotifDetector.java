package org.firstinspires.ftc.teamcode.Subsystems.Motif;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class MotifDetector {

    private final Limelight3A limelight;

    private static final int GPP_ID = 21;
    private static final int PGP_ID = 22;
    private static final int PPG_ID = 23;

    public MotifDetector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags.isEmpty()) {
            MatchMotif.setPattern(MatchMotif.MotifPattern.UNKNOWN);
            return;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            System.out.println("Detected tag ID: " + id); // Debug log

            switch (id) {
                case GPP_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.GPP);
                    return;
                case PGP_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.PGP);
                    return;
                case PPG_ID:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.PPG);
                    return;
                default:
                    MatchMotif.setPattern(MatchMotif.MotifPattern.UNKNOWN);
            }
        }
    }

}
