/*
 * This subsystem is a simple motor that moves the fuel (game piece) to the conveyor. 
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinsterConstants;

public class Spindexer extends SubsystemBase{
    // Declares spinster motors
    private TalonFX m_motor;
    
    // Used to control the speed of the motor
    private DutyCycleOut m_dutyCycle;

    public Spindexer(){
        m_motor = new TalonFX(SpinsterConstants.spinsterID);
        
        // The motor will start with half speed
        m_dutyCycle = new DutyCycleOut(SpinsterConstants.dutyCycle);
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
}
