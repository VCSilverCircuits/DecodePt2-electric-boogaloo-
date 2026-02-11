package org.firstinspires.ftc.teamcode.Subsystems.Motif;

/**
 * Shared state for the match motif pattern.
 * Autonomous sets this once at the start, teleop reads it.
 */
public class MatchMotif {

    public enum MotifPattern {
        GPP, // Green, Purple, Purple
        PGP, // Purple, Green, Purple
        PPG, // Purple, Purple, Green
        UNKNOWN // fallback if auto failed to detect
    }

    private static MotifPattern pattern = MotifPattern.UNKNOWN;

    /** Sets the motif pattern. Called by autonomous after detection. */
    public static void setPattern(MotifPattern p) {
        pattern = p;
    }

    /** Gets the motif pattern. Called by teleop when shooting. */
    public static MotifPattern getPattern() {
        return pattern;
    }

    /** Resets the pattern to UNKNOWN. Optional if needed between matches. */
    public static void reset() {
        pattern = MotifPattern.UNKNOWN;
    }

}
