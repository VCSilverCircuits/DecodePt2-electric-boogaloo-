package org.firstinspires.ftc.teamcode.Motif;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * Detects the match motif using AprilTags at the start of autonomous.
 * Once a tag is detected, the corresponding motif pattern is stored in MatchMotif.
 */
public class MotifDetector {

    private final Limelight3A camera;
    private boolean detected = false;
    private MatchMotif.MotifPattern detectedPattern = MatchMotif.MotifPattern.UNKNOWN;

    // AprilTag IDs for each motif
    private static final int GPP_ID = 21;
    private static final int PGP_ID = 22;
    private static final int PPG_ID = 23;

    public MotifDetector(Limelight3A camera) {
        this.camera = camera;
    }

    /** Call this in a loop during autonomous to continuously check for AprilTags. */
    public void update() {
        if (detected) return; // already detected, no need to check again

        LLResult result = camera.getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : fiducials) {
            int id = tag.getFiducialId();
            switch (id) {
                case GPP_ID:
                    detectedPattern = MatchMotif.MotifPattern.GPP;
                    detected = true;
                    break;
                case PGP_ID:
                    detectedPattern = MatchMotif.MotifPattern.PGP;
                    detected = true;
                    break;
                case PPG_ID:
                    detectedPattern = MatchMotif.MotifPattern.PPG;
                    detected = true;
                    break;
            }
            if (detected) {
                MatchMotif.setPattern(detectedPattern); // lock pattern for teleop
                break;
            }
        }
    }

    /** Returns true if a motif has been detected. */
    public boolean hasDetected() {
        return detected;
    }

    /** Returns the detected pattern, or UNKNOWN if not detected yet. */
    public MatchMotif.MotifPattern getDetectedPattern() {
        return detectedPattern;
    }
}
