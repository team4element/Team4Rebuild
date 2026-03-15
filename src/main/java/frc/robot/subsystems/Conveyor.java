/*
 * This subsystem moves the fuel (game piece) from the spinster into the turret.
 * The conveyor simply runs using a series of rollers.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase{
    // Declare the conveyor motor
    private TalonFX m_motor;

    // Controls the motor's output/speed
    private DutyCycleOut m_dutyCycle;
    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_limitConfig = new CurrentLimitsConfigs();
    private TalonFXConfigurator m_configurator;

    public Conveyor(){
        m_motor = new TalonFX(ConveyorConstants.conveyorID);

        // The motor will start with half speed
        m_dutyCycle = new DutyCycleOut(ConveyorConstants.dutyCycle);

        m_configurator = m_motor.getConfigurator();
        m_limitConfig.StatorCurrentLimit = ConveyorConstants.statorCurrentLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;
        m_configurator.apply(m_limitConfig);
    }

    /**
     * Applies a desired power to the motor
     * @param speedPercentage from -1 to 1
     */
    public void runMotor(double speedPercentage){
        m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
    }

    /*
     * Stops the movement of the motor
     */
    public void stopMotor(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
    }
}
