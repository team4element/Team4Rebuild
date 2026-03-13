package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

    public static final int turretID = 14;
    public static final int shooterLeftID = 15;
    public static final int shooterRightID = 16;

    public static final double leftLimit = 202;
    public static final double rightLimit = -156;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 1;

    public static final double turretStatorLimit = 100;
    public static final double shooterStatorLimit = 100;

    public static final double turretSupplyLimit = 20;
    public static final double shooterSupplyLimit = 20;

    public static final double gearRatio = 11.66667; 

    // Motion Magic Configuration
    public static final double turretMaxVelocity = 60.0;     // Rotations per second at the motor
    public static final double turretMaxAcceleration = 140.0; // Snappy acceleration
    public static final double turretMaxJerk = 800.0;         // S-Curve smoothing to protect the gearbox

    // Refined Feedback/Feedforward
    public static final double KPTurret = 2.0;  
    public static final double KDTurret = 0.1;  
    public static final double KSTurret = 0.25; 
    public static final double KVTurret = 0.11; // 12V / ~105 Max RPS of Kraken/Falcon

    public static final double KPShooter = 0.3; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
    public static final double KVShooter = 0.12;

    public static final double shooterTimeout = 2;
    public static final double distanceUpperLimit = 210;
    public static final double distanceLowerLimit = 49;

    // How fast the ball travels (roughly) to calculate travel time lead
    public static final double estimatedShotVelocity = 15.0; // meters per second

    // Offset of the Limelight lens relative to the center of the TURRET pivot
    // (Used for setCameraPose_RobotSpace)
    public static final double turretToLensForward = 0.1; // meters
    public static final double aimTimeout = 1.5; // In seconds
}
