/*
 * This subsystem collects fuel (game piece) from the floor into the hopper (basket) through rollers attached to the pivot.
 * The pivot could be moved either manually (through controller input) or through setpoints.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    // Declares motors for intake.
    private TalonFX m_leftIntake, m_rightIntake; 
    // Declares motors for the intake's pivot.
    public TalonFX m_leftPivot, m_rightPivot;

    // Used to control the speed of motors.
    private DutyCycleOut m_dutyCyclePivot;
    // Used to control motor's rotation (position) with a given speed.
    private PositionVoltage m_positionRequest;
    private VelocityVoltage m_voltageRequest;

    private TalonFXConfiguration m_pivotLeftConfig;
    private TalonFXConfiguration m_pivotRightConfig;
    private TalonFXConfiguration m_rollerConfig;

    double lastPLeft;
    double lastPRight;
    double m_holdValueLeft;
    double m_holdValueRight;

    boolean debug = false;
    boolean down = false;

    // Initiates objects for linear slide and roller motors.  
    public Intake(){
        m_leftIntake = new TalonFX(IntakeConstants.intakeLeftID);
        m_rightIntake = new TalonFX(IntakeConstants.intakeRightID);

        m_leftPivot = new TalonFX(IntakeConstants.pivotLeftID);
        m_rightPivot = new TalonFX(IntakeConstants.pivotRightID);

        // The pivot and roller motors will start with half speed.
        m_dutyCyclePivot = new DutyCycleOut(IntakeConstants.dutyCyclePivot);

        m_positionRequest = new PositionVoltage(0).withSlot(0);
        m_voltageRequest = new VelocityVoltage(0).withSlot(0);

        // Sets the PID values and reverses motor.
        m_pivotLeftConfig = new TalonFXConfiguration();
        m_pivotRightConfig = new TalonFXConfiguration();

        m_rollerConfig = new TalonFXConfiguration();

        m_pivotLeftConfig.Slot0.kP = IntakeConstants.KPLeft;
        m_pivotLeftConfig.Slot0.kD = IntakeConstants.KDLeft;

        m_pivotRightConfig.Slot0.kP = IntakeConstants.KPRight;
        m_pivotRightConfig.Slot0.kD = IntakeConstants.KDRight;

        m_rollerConfig.Slot0.kP = 0.5;

        m_pivotRightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        m_leftPivot.getConfigurator().apply(m_pivotLeftConfig);
        m_rightPivot.getConfigurator().apply(m_pivotRightConfig);

        m_leftIntake.getConfigurator().apply(m_rollerConfig);

        lastPLeft = IntakeConstants.KPLeft;
        lastPRight = IntakeConstants.KPRight;
        m_holdValueLeft = 12;
        m_holdValueRight = 12;

        SmartDashboard.putNumber("Left Pivot P", IntakeConstants.KPLeft);
        SmartDashboard.putNumber("Left Pivot S", IntakeConstants.KPRight);

        m_leftPivot.setNeutralMode(NeutralModeValue.Brake);
        m_rightPivot.setNeutralMode(NeutralModeValue.Brake);
        m_leftIntake.setNeutralMode(NeutralModeValue.Coast);
        m_rightIntake.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets the starting position for the pivot motor.
     * @param motor to home.
     */
    public void homePivot(TalonFX motor){
        motor.setPosition(0);
    }

    /**
     * This gets the motor's rotation.
     * @param motor for the data.
     * @return motor rotations.
     */
    public double getPivotPosition(TalonFX motor){
        return m_leftPivot.getPosition().getValueAsDouble();
    }

    public void pivotOn(double percentage) {
      //  System.out.println("PERCENT :" + percentage);
        if(getPivotPosition(m_leftPivot) > IntakeConstants.upperPivotLimit && getPivotPosition(m_leftPivot) < IntakeConstants.lowerPivotLimit){
            setPivotPercentage(m_leftPivot, percentage);
            setPivotPercentage(m_rightPivot, percentage);
            if(percentage <= 0){
                runRollers(30);
            }else{
                runRollers(-30);
            }
        } else if(getPivotPosition(m_leftPivot) <= IntakeConstants.upperPivotLimit && percentage > 0 ){ // RT (down)
            setPivotPercentage(m_leftPivot, percentage);
            setPivotPercentage(m_rightPivot, percentage);
        }else if (getPivotPosition(m_leftPivot) >= IntakeConstants.lowerPivotLimit && percentage < 0){ // LT (up)
            setPivotPercentage(m_leftPivot, percentage);
            setPivotPercentage(m_rightPivot, percentage);
        }
    }

    /**
     * This applies a certain power percentage to a motor.
     * @param motor to power.
     * @param percentage from -1 to 1.
     */
    public void setPivotPercentage(TalonFX motor, double percentage){
        motor.setControl(m_dutyCyclePivot.withOutput(percentage));
        m_holdValueLeft = getPivotPosition(m_leftPivot);
        m_holdValueRight = getPivotPosition(m_rightPivot);
    }
    
    /**
     * Runs the pivot motor to extend the intake out of the robot.
     * @param speedPercentage -1 to 1.
    */
    public void extendIntake(TalonFX motor, double speedPercentage){
        if(ControllerConstants.operatorController.rightTrigger().getAsBoolean()){
             double limit = IntakeConstants.lowerPivotLimit;

            if(getPivotPosition(motor) < limit){
                setPivotPercentage(motor, speedPercentage);
        

            }else if(getPivotPosition(motor) >= limit){
                setPivotPercentage(motor, 0);

            }
             m_holdValueLeft = getPivotPosition(m_leftPivot);
             m_holdValueRight = getPivotPosition(m_rightPivot);
        }else if(ControllerConstants.operatorController.leftTrigger().getAsBoolean()){
             double limit = IntakeConstants.upperPivotLimit;

            if(getPivotPosition(motor) <= limit){
                setPivotPercentage(motor, 0);

            }else if(getPivotPosition(motor) > limit){
                setPivotPercentage(motor, -speedPercentage);

            }
            m_holdValueLeft = getPivotPosition(m_leftPivot);
            m_holdValueRight = getPivotPosition(m_rightPivot);
        }else{
            setPivotPercentage(motor, 0);
        }
    }

    public void manualIntake(TalonFX motor, double speedPercentage){
        if(ControllerConstants.operatorController.povDown().getAsBoolean()){
            setPivotPercentage(motor, speedPercentage);
            m_holdValueLeft = getPivotPosition(m_leftPivot);
            m_holdValueRight = getPivotPosition(m_rightPivot);

        }
        else if(ControllerConstants.operatorController.povUp().getAsBoolean()){
            setPivotPercentage(motor, -speedPercentage);
            m_holdValueLeft = getPivotPosition(m_leftPivot);
            m_holdValueRight = getPivotPosition(m_rightPivot);

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

    /**
     * This moves a motor from the pivot of the intake to the desired position based on it's distance from the midpoint.
     * @param motor to use.
     */
    public void automaticPivot(TalonFX motor, double setpoint){
            pivotToSetpoint(motor, setpoint);
            m_holdValueLeft = getPivotPosition(m_leftPivot);
            m_holdValueRight = getPivotPosition(m_rightPivot);
    }

    /**
     * Runs until the rollers reach a desired speed.
     * @param speedPercentage from -1 to 1.
     */
    public void runRollers(double speedPercentage){
        m_leftIntake.setControl(m_voltageRequest.withVelocity(speedPercentage));
        m_rightIntake.setControl(m_voltageRequest.withVelocity(-speedPercentage));
    }

    public void pivotOff(){
        m_leftPivot.set(0);
        m_rightPivot.set(0);
    } 

    /**
     * Stops the intake motors and holds position of the pivot motor.
     */
    public void stopMotors(){
        m_leftPivot.setControl(m_dutyCyclePivot.withOutput(0));
        m_rightPivot.setControl(m_dutyCyclePivot.withOutput(0));
        m_leftIntake.setControl(m_dutyCyclePivot.withOutput(0));
        m_rightIntake.setControl(m_voltageRequest.withVelocity(0));
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

    public void holdPose(){
        pivotToSetpoint(m_leftPivot, m_holdValueLeft);
        pivotToSetpoint(m_rightPivot, m_holdValueRight);
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
