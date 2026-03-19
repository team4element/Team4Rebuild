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

    // Control Requests
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        m_leftMotor = new TalonFX(ShooterConstants.shooterLeftID);
        m_rightMotor = new TalonFX(ShooterConstants.shooterRightID);

        // --- Shooter Motor Configuration ---
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        // PID & Feedforward from ShooterConstants
        shooterConfig.Slot0.kP = ShooterConstants.KPShooter;
        shooterConfig.Slot0.kI = ShooterConstants.KIShooter;
        shooterConfig.Slot0.kD = ShooterConstants.KDShooter;
        shooterConfig.Slot0.kV = ShooterConstants.KVShooter; 

        // Current Limits
        shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.shooterStatorLimit;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.shooterSupplyLimit;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Mechanical Settings
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Apply config to both motors
        m_leftMotor.getConfigurator().apply(shooterConfig);
        m_rightMotor.getConfigurator().apply(shooterConfig);

    

        /* * Set up Follower: Right motor will mimic the Left motor.
         * If your motors are facing each other, one usually needs to be opposed.
         * Set 'opposeMasterDirection' to true if they spin against each other.
         */
        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /**
     * Commands the shooter to a specific velocity.
     */
    public void setRPS(double rps) {
        m_leftMotor.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void stop() {
        m_leftMotor.set(0);
    }

    public double getCurrentRPS() {
        return m_leftMotor.getVelocity().getValueAsDouble();
    }

    public double calculateTargetRPS(double distanceMeters) {
        double distInches = distanceMeters * TurretConstants.metersToInches;
        if (distInches <= 0) return 0;

        // Regression formula
        return (0.000011 * Math.pow(distInches, 3)) - 
               (0.003632 * Math.pow(distInches, 2)) + 
               (0.59999 * distInches) + 32.574475;
    }

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