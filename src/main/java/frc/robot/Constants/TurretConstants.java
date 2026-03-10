package frc.robot.Constants;

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

    public static final double KPTurret = 1.7; // Think of this as the power value.
    public static final double KITurret = 0.0;
    public static final double KDTurret = 0.0; // Damp value, helps with ossilation, if too jerky -> higher value, if too slow -> lower value + more kP
    public static final double KSTurret = 0.8;
    public static final double KVTurret = 0.0;

    public static final double KPShooter = 0.3; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
    public static final double KVShooter = 0.12;

    public static double shooterSpeed = 100;
    public static final double shooterTimeout = 2;
    public static final double distanceUpperLimit = 210;
    public static final double distanceLowerLimit = 49;

    // Hub location on the field in meters.
    public static final Translation2d hubLocation = new Translation2d(4.625, 4);
}
