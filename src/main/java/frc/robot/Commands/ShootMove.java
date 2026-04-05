package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret; // Import Turret
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
        m_fieldLayout = fieldLayout;
        m_drivetrain = drivetrain;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        // Get alliance-specific target
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int targetTagID = (alliance == Alliance.Blue) ? VisionConstants.centerHubBlueTag : VisionConstants.centerHubRedTag;
        
        var hubPose = m_fieldLayout.getTagPose(targetTagID);

        if (!hubPose.isEmpty()) {
            Translation2d hubCenterLocation = hubPose.get().toPose2d().getTranslation();
            
            // Get the compensated virtual target the turret is using
            Translation2d virtualHubLocation = m_turret.calculateVirtualTarget(hubCenterLocation);

            // Calculate the distance from the TURRET to the virtual target
            double virtualDistanceMeters = virtualHubLocation.getDistance(m_turret.getTurretPose().getTranslation());

            // Get the robot's current field-relative velocity (using your drivetrain state)
            var robotSpeeds = m_drivetrain.getState().Speeds; 
            var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, m_drivetrain.getState().Pose.getRotation());
            
            Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

            // Get the vector pointing from the robot directly to the hub
            Translation2d robotToHubVector = hubCenterLocation.minus(m_turret.getTurretPose().getTranslation());
            
            // Project the velocity onto the target vector (Dot Product)
            // This gives us positive if moving TOWARD the target, and negative if moving AWAY!
            double radialVelocityMps = robotVelocity.getX() * (robotToHubVector.getX() / robotToHubVector.getNorm()) +
                                       robotVelocity.getY() * (robotToHubVector.getY() / robotToHubVector.getNorm());


            double targetRPS = m_shooter.shootingDistanceVirtualTarget(virtualDistanceMeters, radialVelocityMps);
            m_shooter.setRPS(targetRPS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Only stop the shooter. Let the turret do its own thing!
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}