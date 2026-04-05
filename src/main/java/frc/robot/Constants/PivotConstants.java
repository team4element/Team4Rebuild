// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class PivotConstants {
    public static final int pivotLeftID = 11;
    public static final int pivotRightID = 10;

    public static final double dutyCycle = 0.5; // Limits the motor to use half speed.

    public static final double KPLeft = 0.1;
    public static final double KDLeft = 0.05;
    public static final double KPRight = 0.1; //0.25
    public static final double KDRight = 0.05; //0,04

    public static final double lowerPivotLimit = 18;
    public static final double upperPivotLimit = 0.0;
    public static final double pivotMidPoint = 9;

    public static final double pivotTimeout = 0.8;    
    public static final double pivotSpeed = 0.4;   
    public static final double poseToIntake = 18; 
    public static final double poseForAuto = 10.5; 
}
