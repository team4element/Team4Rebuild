package frc.robot.Constants;

public class VisionConstants {
    public static final double inchesToMeters = 0.0254;

    public static final double altitudeMeters = 25.5 * inchesToMeters;
    public static final double forwardOffsetMeters = ((27.5)/2) * inchesToMeters; 
    public static final double sideOffsetMeters = 0.1524;
    public static final double mountedDegree = 2; // The degree the limelight is tilted from vertical.
    public static final double hubApriltagHeightMeters = 44 * inchesToMeters; 

    public static final int centerHubBlueTag = 26;
    public static final int centerHubRedTag = 10;
}
