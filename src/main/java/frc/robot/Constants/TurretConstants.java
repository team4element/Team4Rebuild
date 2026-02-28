package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;

public class TurretConstants {

    public static final int turretID = 19;
    public static final int shooterID = 18;

    public static final double leftLimit = -138;
    public static final double rightLimit = 108;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 1;

    public static final double turretStatorLimit = 100;
    public static final double shooterStatorLimit = 100;

    public static final double turretSupplyLimit = 20;
    public static final double shooterSupplyLimit = 20;

    public static final double KPTurret = 3.0; // Think of this as the power value.
    public static final double KITurret = 0.0;
    public static final double KDTurret = 0.1; // Damp value, helps with ossilation, if too jerky -> higher value, if too slow -> lower value + more kP
    public static final double KSTurret = 0.4;
    public static final double KVTurret = 0.0;

    public static final double KPShooter = 0.01; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
    public static final double KVShooter = 0.118;

    public static double shooterSpeed = 95;
    public static final double shooterTimeout = 2;
    public static final double distanceUpperLimit = 145;
    public static final double distanceLowerLimit = 50;

    public static final double turretSpeed = 50;

    // Hub location on the field in meters.
    public static final Translation2d hubLocation = new Translation2d(4.625, 4);
}
