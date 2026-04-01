/*
 * This subsystem collects fuel (game piece) from the floor into the hopper (basket) through rollers attached to the pivot.
 * The pivot could be moved either manually (through controller input) or through setpoints.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    // Declares motors for intake.
    private TalonFX m_leftIntake, m_rightIntake; 

    // Used to control the speed of motors.
    private DutyCycleOut m_dutyCyclePivot;
    private VelocityVoltage m_voltageRequest;

    private TalonFXConfiguration m_config;

    // Initiates objects for linear slide and roller motors.  
    public Intake(){
        m_leftIntake = new TalonFX(IntakeConstants.intakeLeftID);
        m_rightIntake = new TalonFX(IntakeConstants.intakeRightID);

        // The motors will start with half speed.
        m_dutyCyclePivot = new DutyCycleOut(IntakeConstants.dutyCycle);

        m_voltageRequest = new VelocityVoltage(0).withSlot(0);

        m_config.Slot0.kP = IntakeConstants.KPRollers; 

        m_leftIntake.getConfigurator().apply(m_config);
        m_rightIntake.getConfigurator().apply(m_config);

        m_leftIntake.setNeutralMode(NeutralModeValue.Coast);
        m_rightIntake.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Runs until the rollers reach a desired speed.
     * @param speedPercentage from -1 to 1.
     */
    public void runRollers(double speedPercentage){
        m_leftIntake.setControl(m_voltageRequest.withVelocity(speedPercentage));
        m_rightIntake.setControl(m_voltageRequest.withVelocity(-speedPercentage));
    }

    /**
     * Stops the intake motors and holds position of the pivot motor.
     */
    public void stopMotors(){
        m_leftIntake.setControl(m_dutyCyclePivot.withOutput(0));
        m_rightIntake.setControl(m_dutyCyclePivot.withOutput(0));
    }
}
