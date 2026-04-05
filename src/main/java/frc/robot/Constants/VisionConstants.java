package frc.robot.Constants;

public class VisionConstants {

    public static final double inchesToMeters = 0.0254;

    public static final double altitudeMeters = 6.5 * inchesToMeters;
    public static final double forwardOffsetMeters = 12.75 * inchesToMeters; 
    public static final double sideOffsetMeters = 0 * inchesToMeters;
    public static final double mountedDegree = 35.8; // The degree the limelight is tilted from vertical.
    public static final double hubApriltagHeightMeters = 44.5 * inchesToMeters; 
    public static final double turretOffsetX = 5.869 * inchesToMeters;
    public static final double turretOffsetY = 6.0085 * inchesToMeters;
    public static final double cameraRadius = 7.538 * inchesToMeters;

    public static final int centerHubBlueTag = 26;
    public static final int centerHubRedTag = 10;

    public static final double acceptedAvgDistance = 3.0;

    public static final int initialIMUMode = 4;

    public static final String kLimelightName = "limelight-four";
    public static final String kLimelightNameTurret = "limelight-turret";

    //TODO: Tune Values (maybe put these to the dash board)

    /**
     * The maximum acceptable ambiguity for a single-tag read (0.0 to 1.0).
     * High ambiguity means the camera is struggling to tell the tag's true orientation.
     * Lower this value if the robot is "teleporting" when looking at tags from steep angles.
     */
    public static final double kMaxOneTagAmbiguity = 0.7;

    public static final double kMaxOneTagDistanceMeters = 4.5; //  The maximum distance (in meters) the robot can be from a tag before we ignore it.

    /**
     * The max distance (in meters) allowed between the Limelight's estimate and the robot's 
     * current calculated encoder odometry. 
     * If the camera says we suddenly moved x meters in a single frame, we assume it's a false positive.
     */
    public static final double kMaxOdometryDiscrepancyMeters = 2.0;

    /**
     * The maximum allowed age (in seconds) of the camera frame before it is considered stale.
     * If network lag or camera processing takes longer than 0.5 seconds, we drop the frame
     * to avoid injecting heavily outdated data into the pose estimator.
     */
    public static final double kMaxDataAgeSeconds = 0.5;

    /**
     * A scalar applied to the calculated standard deviations (trust factor).
     * Lowering this (e.g., to 0.2) makes the robot trust the camera MORE and snap to it aggressively.
     * Raising this (e.g., to 1.0) makes the robot trust its wheel encoders more than the camera.
     */
    public static final double kBaseTrustScale = 0.01; 

    /**
     * A safety floor (in meters) for the average tag distance calculation.
     * This prevents a scenario where the robot gets so close to a tag that the distance 
     * approaches 0, resulting in a 0 standard deviation that breaks CTRE's matrix math.
     */
    public static final double kMinAvgTagDistFloor = 0.1; 

    /**
     * Represents a near-infinite standard deviation used to ignore a specific axis measurement.
     * We give the camera an insanely high "untrustworthy" value for rotation when only 1 tag 
     * is visible, forcing the drivetrain to rely solely on the rock-solid gyroscope for heading.
     */
    public static final double kUnattainableStdDev = 999999.0;

    /**
     * The baseline translation error (in meters) for a single tag at a 1-meter distance.
     * This forms the foundation of the dynamic standard deviation calculation.
     */
    public static final double kBaselineStdDevMeters = 0.2;

    /**
     * The baseline rotation error (in radians) when viewing a multi-tag layout 
     * from a 1-meter distance. 0.3 radians is roughly 17 degrees.
     */
    public static final double kBaselineRotationStdDevRadians = 0.3;
}
