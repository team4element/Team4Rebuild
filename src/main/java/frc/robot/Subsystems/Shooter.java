/*
 * This subsystem works alongside the turret and odometry to find the RPS needed to get to the target. 
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class Shooter extends SubsystemBase {
    // Hardware
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // Dependencies
    private CommandSwerveDrivetrain m_drivetrain;

    // Control Requests
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Shooter(CommandSwerveDrivetrain drivetrain) {
        m_leftMotor = new TalonFX(ShooterConstants.shooterLeftID);
        m_rightMotor = new TalonFX(ShooterConstants.shooterRightID);

        m_drivetrain = drivetrain;

        // --- Shooter Motor Configuration ---
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        // PID & Feedforward from ShooterConstants
        shooterConfig.Slot0.kP = ShooterConstants.KPShooter;
        shooterConfig.Slot0.kI = ShooterConstants.KIShooter;
        shooterConfig.Slot0.kD = ShooterConstants.KDShooter;
        shooterConfig.Slot0.kV = ShooterConstants.KVShooter; 

        // Mechanical Settings
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        /* * Set up Follower: Right motor will mimic the Left motor.
         * If your motors are facing each other, one usually needs to be opposed.
         * Set 'opposeMasterDirection' to true if they spin against each other.
         */
        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Apply config to both motors
        m_leftMotor.getConfigurator().apply(shooterConfig);
        m_rightMotor.getConfigurator().apply(shooterConfig);
    }

    /**
     * Commands the shooter to a specific RPS.
     * @param rps
     */
    public void setRPS(double rps) {
        m_leftMotor.setControl(m_velocityRequest.withVelocity(rps));
    }

    /*
     * Stops the movement of the motor.
     */
    public void stop() {
        m_leftMotor.set(0);
    }

    /**
     * Gets the current RPS (rotations per second) of the motor.
     * @return RPS.
     */
    public double getCurrentRPS() {
        return m_leftMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Uses the distance from the drivetrain to the hub in a regression formula to find the target RPS.
     * @return the RPS for the shooter to reach our target distance.
     */
    public double shootingDistance() {
        double distMeters = m_drivetrain.getOdometryDistanceMeters();
    
        if (distMeters <= 0) return 0;

        // Convert to inches for the regression formula
        double distInches = distMeters * TurretConstants.metersToInches;

        // Extra inch b/c we are aiming at the target
        double adjustedInches = distInches - 1.0; 

        // Your Regression Formula
        double targetRPS = (0.000011 * Math.pow(adjustedInches, 3)) - 
                       (0.003632 * Math.pow(adjustedInches, 2)) + 
                       (0.59999 * adjustedInches) + 32.574475;
                       
        return targetRPS;
    }

    /**
     * Checks if the motor is within the tolerance of our targetRPS.
     * @param targetRPS
     * @return wheather or not the motor is at target.
     */
    public boolean isAtVelocity(double targetRPS) {
        double tolerance = 1.5; 
        return Math.abs(getCurrentRPS() - targetRPS) < tolerance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Current RPS", getCurrentRPS());
        SmartDashboard.putBoolean("Shooter/At Velocity", isAtVelocity(m_velocityRequest.Velocity));
    }
}