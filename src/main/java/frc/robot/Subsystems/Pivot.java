/*
 * This subsystem controls the pivot on the intake, which can be controlled manually or by motor rotations. 
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

/** Add your docs here. */
public class Pivot extends SubsystemBase{
    // Declares motors for the intake's pivot.
    private TalonFX m_leftPivot, m_rightPivot;

    // Used to control the speed of motors.
    private DutyCycleOut m_dutyCycle;
    // Used to control motor's rotation (position) with a given speed.
    private PositionVoltage m_positionRequest;

    private TalonFXConfiguration m_pivotLeftConfig;
    private TalonFXConfiguration m_pivotRightConfig;

    double lastPLeft;
    double lastPRight;

    public Pivot(){
        m_leftPivot = new TalonFX(PivotConstants.pivotLeftID);
        m_rightPivot = new TalonFX(PivotConstants.pivotRightID);

        // The motors will start with half speed.
        m_dutyCycle = new DutyCycleOut(PivotConstants.dutyCycle);

        m_positionRequest = new PositionVoltage(0).withSlot(0);

        // Sets the PID values and reverses motor.
        m_pivotLeftConfig = new TalonFXConfiguration();
        m_pivotRightConfig = new TalonFXConfiguration();

        m_pivotLeftConfig.Slot0.kP = PivotConstants.KPLeft;
        m_pivotLeftConfig.Slot0.kD = PivotConstants.KDLeft;

        m_pivotRightConfig.Slot0.kP = PivotConstants.KPRight;
        m_pivotRightConfig.Slot0.kD = PivotConstants.KDRight;

        m_pivotRightConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        // Used for placing values onto the SmartDashboard when debugging.
        lastPLeft = PivotConstants.KPLeft;
        lastPRight = PivotConstants.KPRight;

        SmartDashboard.putNumber("Pivot Left P", PivotConstants.KPLeft);
        SmartDashboard.putNumber("Pivot Right P", PivotConstants.KPRight);

        m_leftPivot.setNeutralMode(NeutralModeValue.Brake);
        m_rightPivot.setNeutralMode(NeutralModeValue.Brake);

        m_leftPivot.getConfigurator().apply(m_pivotLeftConfig);
        m_rightPivot.getConfigurator().apply(m_pivotRightConfig);
    }

    /**
     * Sets the starting position for the pivot motor.
     * @param motor to home.
     */
    public void homePivot(){
        m_leftPivot.setNeutralMode(NeutralModeValue.Brake);
        m_rightPivot.setNeutralMode(NeutralModeValue.Brake);
        m_leftPivot.setPosition(0);
    }

    /**
     * This gets the motor's rotation.
     * @param motor for the data.
     * @return motor rotations.
     */
    public double getPivotPosition(){
        return m_leftPivot.getPosition().getValueAsDouble();
    }

    /**
     * This applies a certain power percentage to a motor.
     * @param motor to power.
     * @param percentage from -1 to 1.
     */
    public void setPivotPercentage(double percentage){
        m_leftPivot.setControl(m_dutyCycle.withOutput(percentage));
        m_rightPivot.setControl(m_dutyCycle.withOutput(percentage));
    }
    
    /**
     * Runs the motor until the pivot reaches desired position and set number of rotations.
     * @param motorRotation (desired rotation)
     */
    public void pivotToSetpoint(double motorRotation){
        m_leftPivot.setControl(m_positionRequest.withPosition(motorRotation));
        m_rightPivot.setControl(m_positionRequest.withPosition(motorRotation));
    }

    /**
     * Stops the motor.
     */
    public void stopMotors(){
        m_leftPivot.set(0);
        m_rightPivot.set(0);
    } 

    public void onDisable(){
        m_leftPivot.setNeutralMode(NeutralModeValue.Coast);
        m_rightPivot.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Updates the PID values of the pivot motor.
     */
    public void updateValues(){
        SmartDashboard.putNumber("Current Pose", getPivotPosition());

        double pivotL = SmartDashboard.getNumber("Pivot Left P", PivotConstants.KPLeft);
        double pivotR = SmartDashboard.getNumber("Pivot Right P", PivotConstants.KPRight);

        if(pivotL != lastPLeft || pivotR != lastPRight){
            m_pivotLeftConfig.Slot0.kP = pivotL;
            m_pivotRightConfig.Slot0.kP = pivotR;
            m_leftPivot.getConfigurator().apply(m_pivotLeftConfig);
            m_rightPivot.getConfigurator().apply(m_pivotRightConfig);

            lastPLeft = pivotL;
            lastPRight = pivotR;
        }
    }

}
