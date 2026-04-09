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

import edu.wpi.first.math.MathUtil;
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
    private final com.ctre.phoenix6.controls.NeutralOut m_neutralRequest = new com.ctre.phoenix6.controls.NeutralOut();

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
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Apply config to both motors
        m_leftMotor.getConfigurator().apply(shooterConfig);
        m_rightMotor.getConfigurator().apply(shooterConfig);

        m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
        m_leftMotor.setControl(m_neutralRequest);
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

        //TODO replace with real values
        final int minInches = 0;
        final int maxInches = 500;
        double clampedInches = MathUtil.clamp(adjustedInches, minInches, maxInches);

        // Regression Formula inches to RPS
        double targetRPS = (0.000011 * Math.pow(clampedInches, 3)) -
                           (0.003632 * Math.pow(clampedInches, 2)) +
                           (0.59999 * clampedInches) + 32.574475;

        return targetRPS;
    }

    /**
     * Finds target RPS based on distance AND active robot velocity!
     * @param distMeters Distance to the virtual target.
     * @param radialVelocityMps Speed of the robot moving directly toward the target (+ is toward, - is away).
     */
    public double shootingDistanceVirtualTarget(double distMeters, double radialVelocityMps) {
        if (distMeters <= 0) return 0;

        double distInches = distMeters * TurretConstants.metersToInches;
        double adjustedInches = distInches - 1.0;

        final int minInches = 15;
        final int maxInches = 220;
        double clampedInches = MathUtil.clamp(adjustedInches, minInches, maxInches);

        // Get the base RPS from the regression formula
        double baseRPS = (0.000011 * Math.pow(clampedInches, 3)) -
                         (0.003632 * Math.pow(clampedInches, 2)) +
                         (0.59999 * clampedInches) + 32.574475;

        // Convert radial velocity to a reduction in RPS
        //TODO Tune me
        double velocityCompensationCoefficient = 2;
        double rpsOffset = radialVelocityMps * velocityCompensationCoefficient;

        // Subtract the robot's momentum from the target RPS
        final double bandaid = 0;
        return baseRPS - rpsOffset + bandaid;
    }

    public double getPassRPS(double distMeters, double radialVelocityMps) {
        if (distMeters <= 0) return 0;

        double basePassRPS = 55.0;
        double distanceFactor = 3.3; // RPS increase per meter

        double targetRPS = basePassRPS + (distMeters * distanceFactor);

        double velocityCompensationCoefficient = 2.0;
        double rpsOffset = radialVelocityMps * velocityCompensationCoefficient;

        return MathUtil.clamp(targetRPS - rpsOffset, 0, 108.0);
    }

    /**
     * Checks if the motor is within the tolerance of our targetRPS.
     * @param targetRPS
     * @return whether or not the motor is at target.
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
