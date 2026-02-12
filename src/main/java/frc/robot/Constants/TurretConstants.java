package frc.robot.Constants;

public class TurretConstants {

    public static final int turretID = 19;
    public static final int shooterID = 18;

    public static final double leftLimit = 90;
    public static final double rightLimit = -25;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 1;

    public static final double currentLimitTurret = 80;
    public static final double currentLimitShooter = 80;

    public static final double KPTurret = 0.18; // Think of this as the power value
    public static final double KITurret = 0;
    public static final double KDTurret = 0; // Damp value, helps with ossilation, if too jerky -> higher value, if too slow -> lower value + more kP

    public static final double KPShooter = 0.01; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
    public static final double KVShooter = 0.118;

    public static double shooterSpeed = 95;
    public static final double shooterTimeout = 2;
    public static final double distanceUpperLimit = 145;
    public static final double distanceLowerLimit = 50;

    public static final double turretSpeed = 200;
}
