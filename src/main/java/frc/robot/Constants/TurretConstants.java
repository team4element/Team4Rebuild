package frc.robot.Constants;

public class TurretConstants {

    public static final int turretID = 14;

    public static final double leftLimit = 202;
    public static final double rightLimit = -156;

    public static final double dutyCycleTurret = 0.5;

    public static final double turretStatorLimit = 50;

    public static final double turretSupplyLimit = 80;

    public static final double gearRatio = 11.66667; 

    // Motion Magic Configuration
    public static final double turretMaxVelocity = 60.0;     // Rotations per second at the motor.
    public static final double turretMaxAcceleration = 140.0; // Snappy acceleration.
    public static final double turretMaxJerk = 800.0;         // S-Curve smoothing to protect the gearbox.

    // Refined Feedback/Feedforward
    public static final double KPTurret = 3.8;
    public static final double KDTurret = 0.1;
    public static final double KSTurret = 0.25; 
    public static final double KVTurret = 0.11; // 12V / ~105 Max RPS of Kraken/Falcon /0.11

    public static final double inchesToMeters = 0.0254;
    public static final double metersToInches = 39.3701;

    public static final double distanceUpperLimit = 210 * inchesToMeters;
    public static final double distanceLowerLimit = 49 * inchesToMeters;

    public static final double leftCornerRotation = -0.118; 
    public static final double rightCornerRotation = 0.118; 

    public static final double robotCenterToTurretForward = 0.156;
    public static final double robotCenterToTurretRight = 0.156;   

    public static final double hubNetRadius = 0.65;  //TODO REAL VALUE
    public static final double minShotClearanceDeg = 15.0;  // TODO REAL VALUE
}