/*
 * This subsystem collects fuel (game piece) from the floor into the hopper (basket) through rollers attached to the pivot.
 * The pivot could be moved either manually (through controller input) or through setpoints.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    // Declares motors for intake.
    private TalonFX m_leftIntake, m_rightIntake; 
    // Declares motors for the intake's pivot;
    public TalonFX m_leftPivot, m_rightPivot;

    // Used to control the speed of motors
    private DutyCycleOut m_dutyCyclePivot, m_dutyCycleRollers;
    // Used to control motor's rotation (position) with a given speed
    private PositionVoltage m_positionRequest;
    // Used to follow each motors.
    private TalonFXConfigurator m_leftPivotConfigurator;
    private TalonFXConfigurator m_rightPivotConfigurator;

    private TalonFXConfigurator m_leftIntakeConfigurator;
    private TalonFXConfigurator m_rightIntakeConfigurator;

    private TalonFXConfiguration m_pivotLeftConfig;
    private TalonFXConfiguration m_pivotRightConfig;

    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_slideLimitConfig, m_rollerLimitConfig;

    double lastPLeft;
    double lastPRight;

    boolean debug = false;
    boolean down = false;

    // Initiates objects for linear slide and roller motors.  
    public Intake(){
        m_leftIntake = new TalonFX(IntakeConstants.intakeLeftID);
        m_rightIntake = new TalonFX(IntakeConstants.intakeRightID);

        m_leftPivot = new TalonFX(IntakeConstants.pivotLeftID);
        m_rightPivot = new TalonFX(IntakeConstants.pivotRightID);

        // Linear slide and roller motors will start with half speed.
        m_dutyCyclePivot = new DutyCycleOut(IntakeConstants.dutyCyclePivot);
        m_dutyCycleRollers = new DutyCycleOut(IntakeConstants.dutyCycleRollers);

        m_positionRequest = new PositionVoltage(0).withSlot(0);

        // Sets the PID values and reverses motor.
        m_pivotLeftConfig = new TalonFXConfiguration();
        m_pivotRightConfig = new TalonFXConfiguration();

        m_pivotLeftConfig.Slot0.kP = IntakeConstants.KPLeft;

        m_pivotRightConfig.Slot0.kP = IntakeConstants.KPRight;
        m_pivotRightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        m_leftPivot.getConfigurator().apply(m_pivotLeftConfig);
        m_rightPivot.getConfigurator().apply(m_pivotRightConfig);

        // // Applies current limits.
        // m_slideLimitConfig = new CurrentLimitsConfigs();
        // m_rollerLimitConfig = new CurrentLimitsConfigs();

        // m_slideLimitConfig.StatorCurrentLimit = IntakeConstants.currentLimitPivot;
        // m_slideLimitConfig.StatorCurrentLimitEnable = true;

        // m_rollerLimitConfig.StatorCurrentLimit = IntakeConstants.currentLimitRollers;
        // m_rollerLimitConfig.StatorCurrentLimitEnable = true;

        // This is used to declare the leader and follower for the pivot and intake.
        m_leftIntakeConfigurator = m_leftIntake.getConfigurator();
        m_rightIntakeConfigurator = m_rightIntake.getConfigurator();

        // Creates a leader and follower
        m_rightIntake.setControl(new Follower(IntakeConstants.intakeLeftID, MotorAlignmentValue.Opposed));

        lastPLeft = IntakeConstants.KPLeft;
        lastPRight = IntakeConstants.KPRight;

        SmartDashboard.putNumber("Left Pivot P", IntakeConstants.KPLeft);
        SmartDashboard.putNumber("Left Pivot S", IntakeConstants.KPRight);

        m_leftPivot.setNeutralMode(NeutralModeValue.Brake);
        m_leftIntake.setNeutralMode(NeutralModeValue.Brake);
        m_rightPivot.setNeutralMode(NeutralModeValue.Brake);
        m_rightIntake.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the starting position for the pivot motor.
     */
    public void resetPivot(TalonFX motor){
        motor.setPosition(0);
    }

    public double getPivotPosition(TalonFX motor){
        return motor.getPosition().getValueAsDouble();
    }

    public void setPivotPercentage(TalonFX motor, double percentage){
        motor.setControl(m_dutyCyclePivot.withOutput(percentage));
    }
    
    /**
     * Runs the pivot motor to extend the intake out of the robot.
     * @param speedPercentage -1 to 1.
    */
    public void  extendIntake(TalonFX motor, double speedPercentage){
        if(ControllerConstants.operatorController.leftTrigger().getAsBoolean()){
             double limit = IntakeConstants.lowerPivotLimit;

            if(getPivotPosition(motor) < limit){
                setPivotPercentage(motor, speedPercentage);

            }else if(getPivotPosition(motor) >= limit){
                setPivotPercentage(motor, 0);

            }
        }else if(ControllerConstants.operatorController.rightTrigger().getAsBoolean()){
             double limit = IntakeConstants.upperPivotLimit;

            if(getPivotPosition(motor) <= limit){
                setPivotPercentage(motor, 0);

            }else if(getPivotPosition(motor) > limit){
                setPivotPercentage(motor, -speedPercentage);

            }
        }else{
            setPivotPercentage(motor, 0);
        }
    }

    /**
     * Runs the motor until the pivot reaches desired position and set number of rotations.
     * @param motorRotation (desired rotation)
     */
    public void pivotToSetpoint(TalonFX motor, double motorRotation){
        motor.setControl(m_positionRequest.withPosition(motorRotation));
    }

    public void automaticPivot(TalonFX motor){
        if(getPivotPosition(motor) <= IntakeConstants.pivotMidPoint){
            motor.setControl(m_positionRequest.withPosition(18));
        }else if(ControllerConstants.operatorController.povDown().getAsBoolean()){
            motor.setControl(m_positionRequest.withPosition(16));
        }else if(getPivotPosition(motor) > IntakeConstants.pivotMidPoint){
            motor.setControl(m_positionRequest.withPosition(0));
        }
    }

    /**
     * Runs until the rollers reach a desired speed.
     * @param speedPercentage from -1 to 1.
     */
    public void runRollers(double speedPercentage){
        m_leftIntake.setControl(m_dutyCycleRollers.withOutput(speedPercentage));
    }

    /**
     * Stops the intake motors and holds position of the pivot motor.
     */
    public void stopMotors(){
        m_leftPivot.setControl(m_dutyCyclePivot.withOutput(0));
        m_rightPivot.setControl(m_dutyCyclePivot.withOutput(0));
        m_leftIntake.setControl(m_dutyCyclePivot.withOutput(0));

        m_leftPivot.setNeutralMode(NeutralModeValue.Brake);
        m_leftIntake.setNeutralMode(NeutralModeValue.Brake);
        m_rightPivot.setNeutralMode(NeutralModeValue.Brake);
        m_rightIntake.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Updates the PID values of the pivot motor.
     * @param P
     * @param D
     * @param S
     */
    public void updateValues(double L, double R){
        if(L != lastPLeft || R != lastPRight){
            m_pivotLeftConfig.Slot0.kP = L;
            m_pivotRightConfig.Slot0.kP = R;
            m_leftPivot.getConfigurator().apply(m_pivotLeftConfig);
            m_rightPivot.getConfigurator().apply(m_pivotRightConfig);

            lastPLeft = L;
            lastPRight = R;
        }
    }

    public void periodic(){
        if(debug){
            double pivotL = SmartDashboard.getNumber("Pivot Left P", IntakeConstants.KPLeft);
            double pivotR = SmartDashboard.getNumber("Pivot Right P", IntakeConstants.KPRight);
    
            updateValues(pivotL, pivotR);
        }

        if(m_leftPivot.getPosition().getValueAsDouble() > 17.5){
            down = true;
        }else{
            down = false;
        }

        SmartDashboard.putBoolean("Pivot lowered", down);
    }
}
