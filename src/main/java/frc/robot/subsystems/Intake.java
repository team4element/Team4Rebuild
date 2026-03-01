/*
 * This subsystem collects fuel (game piece) from the floor into the hopper (basket) through rollers attached to the linear slide.
 * The linear slide could be moved either manually (through controller input) or through setpoints.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    // Declares motors.
    private TalonFX m_linearSlide, m_rollers; 

    // Used to control the speed of motors
    private DutyCycleOut m_dutyCycleSlide, m_dutyCycleRollers;
    // Used to control motor's rotation (position) with a given speed
    private PositionVoltage m_positionRequest;
    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_slideLimitConfig, m_rollerLimitConfig;

    // Initiates objects for linear slide and roller motors.  
    public Intake(){
        m_linearSlide = new TalonFX(IntakeConstants.linearSlideID);
        m_rollers = new TalonFX(IntakeConstants.rollersID);

        // Linear slide and roller motors will start with half speed.
        m_dutyCycleSlide = new DutyCycleOut(IntakeConstants.dutyCycleLinearSlide);
        m_dutyCycleRollers = new DutyCycleOut(IntakeConstants.dutyCycleRollers);

        m_positionRequest = new PositionVoltage(0);

        m_slideLimitConfig = new CurrentLimitsConfigs();
        m_rollerLimitConfig = new CurrentLimitsConfigs();

        m_slideLimitConfig.StatorCurrentLimit = IntakeConstants.currentLimitLinearSlide;
        m_slideLimitConfig.StatorCurrentLimitEnable = true;

        m_rollerLimitConfig.StatorCurrentLimit = IntakeConstants.currentLimitRollers;
        m_rollerLimitConfig.StatorCurrentLimitEnable = true;
    }

    /**
     * Sets the starting position for the linear slide motor.
     */
    public void resetLinearSlide(){
        m_linearSlide.setPosition(0);
    }
    
    /**
     * Runs the linear slide motor to extend the intake out of the robot.
     * @param speedPercentage -1 to 1.
    */
    public void extendLinearSlideManual(double speedPercentage){
        m_linearSlide.setControl(m_dutyCycleSlide.withOutput(speedPercentage));
    }

    /**
     * Runs the linear slide motor to retract the intake into the robot.
     * @param speedPercentage -1 to 1.
     */
    public void retractLinearSlideManual(double speedPercentage){
        m_linearSlide.setControl(m_dutyCycleSlide.withOutput(-speedPercentage));
    }

    /**
     * Runs the motor until the linear slide reaches desired position and set number of rotations.
     * @param motorRotation (desired rotation)
     */
    public void linearSlideToSetpoint(double motorRotation){
        m_linearSlide.setControl(m_positionRequest.withPosition(motorRotation));
    }

    /**
     * Stops the intake motors and holds position of the linear slide motor.
     */
    public void stopMotors(){
        m_linearSlide.setControl(m_dutyCycleSlide.withOutput(0));
        m_rollers.setControl(m_dutyCycleRollers.withOutput(0));
        m_linearSlide.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs until the rollers reach a desired speed.
     * @param speedPercentage from -1 to 1.
     */
    public void runRollers(double speedPercentage){
        m_rollers.setControl(m_dutyCycleRollers.withOutput(speedPercentage));
    }
}
