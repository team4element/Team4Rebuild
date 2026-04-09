package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class ShootMove extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagFieldLayout m_fieldLayout;

    public ShootMove(Shooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout fieldLayout) {
        m_shooter = shooter;
        m_turret = turret;
        m_drivetrain = drivetrain;
        m_fieldLayout = fieldLayout;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        // Identify Target based on Alliance
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int targetTagID = (alliance == Alliance.Blue) ? VisionConstants.centerHubBlueTag : VisionConstants.centerHubRedTag;
    
        var hubPose = m_fieldLayout.getTagPose(targetTagID);
        if (!hubPose.isEmpty()) {
            Translation2d hubCenterLocation = hubPose.get().toPose2d().getTranslation();
        
            // Get the compensated virtual target the turret is using for the "Lead"
            Translation2d virtualHubLocation = m_turret.calculateVirtualTarget(hubCenterLocation);
        
            // Calculate distance from Turret to the Virtual Target for the regression
            double virtualDistanceMeters = virtualHubLocation.getDistance(m_turret.getTurretPose().getTranslation());
        
            // Convert Robot-Relative Speeds to Field-Relative Speeds
            var state = m_drivetrain.getState();
            Rotation2d robotRotation = state.Pose.getRotation();
            double fieldVx = state.Speeds.vxMetersPerSecond * robotRotation.getCos() 
                           - state.Speeds.vyMetersPerSecond * robotRotation.getSin();
            double fieldVy = state.Speeds.vxMetersPerSecond * robotRotation.getSin() 
                           + state.Speeds.vyMetersPerSecond * robotRotation.getCos();
        
            // Tangential velocity from robot rotation (turret pivot traces an arc when spinning)
            double omega = state.Speeds.omegaRadiansPerSecond;
            Translation2d robotToTurret = new Translation2d(
                TurretConstants.robotCenterToTurretForward,
                -TurretConstants.robotCenterToTurretRight
            );
            double tangentialVx = omega * robotToTurret.getY();
            double tangentialVy = -omega * robotToTurret.getX();
            Translation2d tangentialVelocityField = new Translation2d(tangentialVx, tangentialVy)
                                                    .rotateBy(robotRotation);
        
            double totalVx = fieldVx + tangentialVelocityField.getX();
            double totalVy = fieldVy + tangentialVelocityField.getY();
        
            Translation2d fieldVelocity = new Translation2d(totalVx, totalVy);
        
            // Calculate Radial Velocity (project total velocity onto the turret-to-hub direction)
            Translation2d robotToHubVector = hubCenterLocation.minus(m_turret.getTurretPose().getTranslation());
            Rotation2d angleToHub = robotToHubVector.getAngle();
        
            double radialVelocityMps = fieldVelocity.getX() * angleToHub.getCos() 
                                     + fieldVelocity.getY() * angleToHub.getSin();
        
            // Calculate and Set Target RPS
            double targetRPS = m_shooter.shootingDistanceVirtualTarget(virtualDistanceMeters, radialVelocityMps);
            m_shooter.setRPS(targetRPS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}