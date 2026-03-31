package frc.robot.Constants;

public class VisionConstants {
    public static final double inchesToMeters = 0.0254;

    public static final double altitudeMeters = 25.5 * inchesToMeters;
    public static final double forwardOffsetMeters = ((27.5)/2) * inchesToMeters; 
    public static final double sideOffsetMeters = 0.1524;
    public static final double mountedDegree = 2; // The degree the limelight is tilted from vertical.
    public static final double hubApriltagHeightMeters = 44 * inchesToMeters; 
    public static final double turretOffsetX = 5.869 * inchesToMeters;
    public static final double turretOffsetY = 6.0085 * inchesToMeters;
    public static final double cameraRadius = 7.538 * inchesToMeters;

    public static final int centerHubBlueTag = 26;
    public static final int centerHubRedTag = 10;

    public static final double acceptedAvgDistance = 3.0;

    public static final int initialIMUMode = 4;
}
