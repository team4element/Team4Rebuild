package frc.robot.Constants;

public class IntakeConstants {

    public static final int pivotLeftID = 11;
    public static final int pivotRightID = 10;
    public static final int intakeLeftID = 8;
    public static final int intakeRightID = 9;

    public static final double dutyCyclePivot = 0.5; //limits the motor to use half speed
    public static final double dutyCycleRollers = 0.5; //limits the motor to use half speed

    public static final double KPLeft = 0.5;
    public static final double KPRight = 0.47;
    
    public static final int currentLimitPivot = 80;    
    public static final int currentLimitRollers = 80; 

    public static final double lowerPivotLimit = 18.5;
    public static final double upperPivotLimit = 0.0;
    public static final double pivotMidPoint = 9.51;

    public static final double linearPivotSpeed = 0.5;    
    public static final double rollerSpeed = 0.5;    
    public static final double intakeTimeout = 2;
}
