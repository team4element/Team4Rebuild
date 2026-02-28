/*
 * This subsystem controls the belt on the climb to pull onto levels 1, 2, and 3.
 * The climb can be used manually or by the internal encoder position.
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase{
    // Declare the climb motor
    private TalonFX m_motor;

    // Declare the vertical bounds based on encoder 
    private double highLimit = 100;
    private double lowLimit = -100;

    // Used to control speed of the motor
    private DutyCycleOut m_dutyCycle;
    // Used to control the PID and motor's default action
    private TalonFXConfiguration m_talonConfigs;
    // Used to control the position of the motor
    private PositionVoltage m_positionRequest;
    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_currentLimit;

    // Controls print statement output to terminal
    private boolean debug;

    public Climb(){
        m_motor = new TalonFX(ClimbConstants.climbID);

        // The turret and shooter motor will start with half speed
        m_dutyCycle = new DutyCycleOut(ClimbConstants.dutyCycle);
        m_currentLimit = new CurrentLimitsConfigs();

        m_talonConfigs = new TalonFXConfiguration();
        m_talonConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        m_talonConfigs.Slot0.kP = ClimbConstants.KP;
        m_talonConfigs.Slot0.kD = ClimbConstants.KD;

        m_positionRequest = new PositionVoltage(0).withSlot(0);

        m_currentLimit.StatorCurrentLimit = ClimbConstants.currentLimit;
        m_currentLimit.StatorCurrentLimitEnable = true;

        debug = false; // This should be true when testing
    }

    // Constantly gets the motor's rotation while testing.
   // public void periodic(){
    //    if(debug == true){
    //    double currentRotation = m_motor.getPosition().getValueAsDouble();
   //     //System.out.println("CLIMB POSITION: " + currentRotation); 

   //     }
   // }

    /*
     * Initiates motor starting position.
     */
    public void resetMotor(){
        m_motor.setPosition(0);
    }

    /*
     * Stops the motor and keeps the climb at current position.
     */
    public void hold(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs the motor in reverse to pull the robot up until it reaches the hook's physical limit.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotorDown(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == lowLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(-speedPercentage));
        }
    }

    /**
     * Runs the motor to reach to the next level until it reaches the hook's physical limit.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotorUp(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == highLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
        }
    }

    /**
     * Assigns a goal value for the climb position to reach next level. 
     * @param setpoint based on motor rotation.
     */
    public void goToSetpoint(double setpoint){
        m_motor.setControl(m_positionRequest.withPosition(setpoint));
    }
}
