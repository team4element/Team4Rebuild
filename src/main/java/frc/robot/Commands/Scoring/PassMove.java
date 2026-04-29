package frc.robot.Commands.Scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
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
        Translation2d target;
        if (alliance == Alliance.Blue) {
            target = (m_drivetrain.getState().Pose.getY() >= VisionConstants.fieldCenterY) ?
                VisionConstants.BLUE_PASS_LEFT :
                VisionConstants.BLUE_PASS_RIGHT;
        } else {
            target = (m_drivetrain.getState().Pose.getY() >= VisionConstants.fieldCenterY) ?
                VisionConstants.RED_PASS_LEFT :
                VisionConstants.RED_PASS_RIGHT;
        }

        // Calculate the Virtual Target for movement compensation
        Translation2d virtualTarget = m_turret.calculateVirtualTarget(target);
        double virtualDist = virtualTarget.getDistance(m_turret.getTurretPose().getTranslation());

        // Calculate total field velocity including tangential component from rotation
        var state = m_drivetrain.getState();
        Rotation2d robotRotation = state.Pose.getRotation();

        double fieldVx = state.Speeds.vxMetersPerSecond * robotRotation.getCos()
                       - state.Speeds.vyMetersPerSecond * robotRotation.getSin();
        double fieldVy = state.Speeds.vxMetersPerSecond * robotRotation.getSin()
                       + state.Speeds.vyMetersPerSecond * robotRotation.getCos();

        // Tangential velocity from robot rotation
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

        // Calculate Radial Velocity (project onto turret-to-target direction)
        Translation2d turretToTargetVector = target.minus(m_turret.getTurretPose().getTranslation());
        double radialVelocityMps = fieldVelocity.getX() * (turretToTargetVector.getX() / turretToTargetVector.getNorm())
                                 + fieldVelocity.getY() * (turretToTargetVector.getY() / turretToTargetVector.getNorm());

        // Command the Shooter
        double targetRPS = m_shooter.getPassRPS(virtualDist, radialVelocityMps);
        m_shooter.setRPS(targetRPS);
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
