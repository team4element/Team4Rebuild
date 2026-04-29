package frc.robot.Constants;

public class PivotConstants {

    // --- MOTOR IDS ---
    public static final int pivotLeftID = 11;
    public static final int pivotRightID = 10;

    public static final double dutyCycle = 0.5; // Limits the motor to use half speed.

    // --- PID CONSTANTS ---
    public static final double KPLeft = 0.55;
    public static final double KDLeft = 0.1;

    public static final double KPRight = 0.52;
    public static final double KDRight = 0.1; 

    // --- PHYSICAL ROTATION LIMITS ---
    public static final double lowerPivotLimit = 18;
    public static final double upperPivotLimit = 0.5;
    public static final double pivotMidPoint = 9;

    // --- AUTON CONSTANTS
    public static final double autoTimeout = 1;  
    public static final double longAutoTimeout = 1.5;    
    
    public static final double poseAutoMiddle = 7; 
    public static final double poseAutoLower = 3; 

    // --- OTHER ---
    public static final double pivotSpeed = 0.3;   
    public static final double poseToIntake = 18.6; 

}
