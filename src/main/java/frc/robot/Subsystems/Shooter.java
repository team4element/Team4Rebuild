package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {
    // Hardware
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // Dependencies
    private CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagFieldLayout m_field_layout;

    // Control Requests
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    private int m_visionLostCounter = 0;
    private int kVisionThreshold = 5;

    public Shooter(AprilTagFieldLayout field_layout, CommandSwerveDrivetrain drivetrain) {
        m_leftMotor = new TalonFX(ShooterConstants.shooterLeftID);
        m_rightMotor = new TalonFX(ShooterConstants.shooterRightID);

        m_drivetrain = drivetrain;
        m_field_layout = field_layout;

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

    /**
    * ONE function to rule them all. 
    * Returns distance in METERS using Vision (primary) or Odometry (fallback).
    */
    public double getBestDistanceMeters() {
        boolean seeTag = LimelightHelpers.getTV("limelight-four");
        double distMeters = 0;

        // 1. Update Flicker Protection
        if (seeTag) {
            m_visionLostCounter = 0;
        } else {
            m_visionLostCounter++;
        }

        // 2. Determine Distance Source
        if (seeTag && m_visionLostCounter < kVisionThreshold) {
            // --- VISION STRATEGY ---
            double[] pose = NetworkTableInstance.getDefault()
                .getTable("limelight-four")
                .getEntry("targetpose_robotspace")
                .getDoubleArray(new double[0]);

            if (pose.length >= 6) {
                // Targetpose_robotspace: [x, y, z, roll, pitch, yaw]
                double x = pose[0];
                double z = pose[2];
                distMeters = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
            } else {
                distMeters = getOdometryDistanceMeters();
            }
        } else {
            // --- ODOMETRY STRATEGY ---
            distMeters = getOdometryDistanceMeters();
        }

        // 3. Final Validation
        if (distMeters > TurretConstants.distanceUpperLimit || distMeters < TurretConstants.distanceLowerLimit) {
            // If out of physical range, default to Odometry as a safety check
            return getOdometryDistanceMeters(); 
        }
        return distMeters;
    }

    private double getOdometryDistanceMeters() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        int targetTagID = (alliance == Alliance.Blue) ? 
                    VisionConstants.centerHubBlueTag : 
                    VisionConstants.centerHubRedTag;

        var hubPoseEntry = m_field_layout.getTagPose(targetTagID);
        
        if (hubPoseEntry.isEmpty()) return 0.0;

        // Get the actual Tag Position
        Translation2d tagLocation = hubPoseEntry.get().toPose2d().getTranslation();

        double centerOffsetMeters = 0.65; // Adjust this based on your specific goal depth
        double xOffset = (alliance == Alliance.Blue) ? -centerOffsetMeters : centerOffsetMeters;

        Translation2d hubCenterLocation = new Translation2d(
            tagLocation.getX() + xOffset,
            tagLocation.getY() // Keep Y the same if the tag is centered on the goal
        );

        // Calculate distance to the VIRTUAL center, not the physical tag
        return robotPose.getTranslation().getDistance(hubCenterLocation);
    }

    public double shootingDistance() {
        double distMeters = getBestDistanceMeters();
    
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
        SmartDashboard.putNumber("distance", getBestDistanceMeters());
    }
}