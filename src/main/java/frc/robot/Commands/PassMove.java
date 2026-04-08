package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;

public class PassMove extends Command {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final CommandSwerveDrivetrain m_drivetrain;

    public PassMove(Shooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain) {
        m_shooter = shooter;
        m_turret = turret;
        m_drivetrain = drivetrain;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d target = (alliance == Alliance.Blue) ? 
                               VisionConstants.BLUE_PASS_TARGET : VisionConstants.RED_PASS_TARGET;

        // 1. Calculate the Virtual Target for movement compensation
        Translation2d virtualTarget = m_turret.calculateVirtualTarget(target);
        double virtualDist = virtualTarget.getDistance(m_turret.getTurretPose().getTranslation());

        // 2. Calculate Radial Velocity (Dot Product)
        var robotSpeeds = m_drivetrain.getState().Speeds; 
        var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, m_drivetrain.getState().Pose.getRotation());
        Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d robotToTargetVector = target.minus(m_turret.getTurretPose().getTranslation());
        
        double radialVelocityMps = robotVelocity.getX() * (robotToTargetVector.getX() / robotToTargetVector.getNorm()) +
                                   robotVelocity.getY() * (robotToTargetVector.getY() / robotToTargetVector.getNorm());

        // 3. Command the Shooter
        double targetRPS = m_shooter.getPassRPS(virtualDist, radialVelocityMps);
        m_shooter.setRPS(targetRPS);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }
}