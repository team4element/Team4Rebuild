// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TunerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackWhileMove extends Command {
  /** Creates a new TrackWhileMove. */
  Turret m_turret;
  CommandSwerveDrivetrain m_drivetrain;

  public TrackWhileMove(Turret turret, CommandSwerveDrivetrain drivetrain) {
    m_turret = turret;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotCurrentPose = m_drivetrain.getState().Pose;
    Pose2d targetPose = new Pose2d(4.62, 4.05, new Rotation2d(Math.PI));
    double speedX = m_drivetrain.getState().Speeds.vxMetersPerSecond;
    double speedY = m_drivetrain.getState().Speeds.vyMetersPerSecond;

    SwerveDrivePoseEstimator poseEstimate = m_drivetrain.m_poseEstimator;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
