/*
 * This subsystem has 6 slots where the fuel (game piece) will move through to get to the conveyor. 
 * The spinster can be controlled manually or through by the slots (60 degrees)
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinsterConstants;

public class Spinster extends SubsystemBase{
    // Declares spinster motors
    public TalonFX m_motor;
    
    // Used to control the speed of the motor
    public DutyCycleOut m_dutyCycle;
    // Used to control the position of the motor
    private PositionVoltage m_positionRequest;
    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    public CurrentLimitsConfigs m_currentLimit;

    public Spinster(){
        m_motor = new TalonFX(SpinsterConstants.spinsterID);
        
        // The motor will start with half speed
        m_dutyCycle = new DutyCycleOut(SpinsterConstants.dutyCycle);
        m_positionRequest = new PositionVoltage(0);
        m_currentLimit = new CurrentLimitsConfigs();
    
        m_currentLimit.StatorCurrentLimit = 100;
        m_currentLimit.StatorCurrentLimitEnable = true;
    }

    /**
     * Applies a desired power to the motor.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotor(double speedPercentage){
        m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
    }

    /*
     * Stops the movement of the motor. 
     */
    public void stopMotor(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
    }

    /*
     * Rotates one of six slots to align with the conveyor. 
     */
    public void rotateSlot(){
        // The degree was found through 360 (degrees in a circle) /6 (the amount of slots)
        m_motor.setControl(m_positionRequest.withPosition(60));
    }
}
