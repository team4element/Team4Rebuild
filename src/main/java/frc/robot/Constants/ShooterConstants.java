package frc.robot.Constants;

public class ShooterConstants {

    // --- MOTOR IDS ---
    public static final int shooterLeftID = 15;
    public static final int shooterRightID = 16;

    // --- PID CONSTANTS ---
    public static final double KPShooter = 0.3; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
    public static final double KVShooter = 0.12; 

    // --- AUTON CONSTANTS ---
    public static final double autoTimeout = 2;
    public static final double longAutoTimeout = 12;

    // Used to conpensate for tracking and moving
    public static final double kAverageBallVelocityMps = 3.2;
}