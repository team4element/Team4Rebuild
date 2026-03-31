/*
 * This subsystem has 6 slots where the fuel (game piece) will move through to get to the conveyor. 
 * The spinster can be controlled manually or through by the slots (60 degrees)
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinsterConstants;

public class Spinster extends SubsystemBase{
    // Declares spinster motors
    private TalonFX m_motor;
    
    // Used to control the speed of the motor
    private DutyCycleOut m_dutyCycle;

    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_currentLimit;
    private TalonFXConfigurator m_configurator;

    public Spinster(){
        m_motor = new TalonFX(SpinsterConstants.spinsterID);
        
        // The motor will start with half speed
        m_dutyCycle = new DutyCycleOut(SpinsterConstants.dutyCycle);
        m_currentLimit = new CurrentLimitsConfigs();

        // m_configurator = m_motor.getConfigurator();
    
        // m_currentLimit.StatorCurrentLimit = 50;
        // m_currentLimit.StatorCurrentLimitEnable = true;
        // m_currentLimit.SupplyCurrentLimit = 80;
        // m_currentLimit.SupplyCurrentLimitEnable = true;

        // m_configurator.apply(m_currentLimit);
    }

    /**
     * Applies a desired power to the motor.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotor(double speedPercentage){

       // System.out.println(speedPercentage);
        m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
    }

    /*
     * Stops the movement of the motor. 
     */
    public void stopMotor(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
    }
}
