package frc.robot.Constants;

public class IntakeConstants {

    public static final int pivotLeftID = 11;
    public static final int pivotRightID = 10;
    public static final int intakeLeftID = 8;
    public static final int intakeRightID = 9;

    public static final double dutyCyclePivot = 0.5; // Limits the motor to use half speed.
    public static final double dutyCycleRollers = 1; // Limits the motor to use half speed.

    public static final double KPLeft = 0.65;
    public static final double KDLeft = 0.05;
    public static final double KPRight = 0.25;
    public static final double KDRight = 0.04;
    
    public static final int currentLimitPivot = 30;    
    public static final int currentLimitRollers = 60; 

    public static final double lowerPivotLimit = 18;
    public static final double upperPivotLimit = 0.0;
    public static final double pivotMidPoint = 9;

    public static final double pivotTimeout = 0.8;    
    public static final double pivotSpeed = 0.3;    
    public static final double intakeSpeed = .75;      
    public static final double intakeTimeout = 2;
}
