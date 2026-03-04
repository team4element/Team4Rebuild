// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackWhileMove extends Command {
  /** Creates a new TrackWhileMove. */
  Turret m_turret;
  CommandSwerveDrivetrain m_drivetrain;
  SwerveDrivePoseEstimator poseEstimate;
  Pose2d robotCurrentPose;
  double TX;
  double currentTarget;
  double timestamp;
  boolean hasTarget;

  public TrackWhileMove(Turret turret, CommandSwerveDrivetrain drivetrain) {
    m_turret = turret;
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is meant to calculate the time the limelight is currently at.

    //timestamp = Timer.getFPGATimestamp() - ((LimelightHelpers.getLatency_Pipeline("limelight-four")/1000));
   // poseEstimate = new SwerveDrivePoseEstimator(
      //m_drivetrain.getKinematics(),
      // m_drivetrain.getGyroAngle(),
      // m_drivetrain.getState().ModulePositions,
      // robotCurrentPose
    //);
   // poseEstimate.addVisionMeasurement(robotCurrentPoseMT2, timestamp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = -LimelightHelpers.getTX("limelight-four");

    // This allows for accurate position based on alliance using MegaTag2.
    if(DriverStation.getAlliance().get() == Alliance.Red){
      robotCurrentPose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-four").pose;
    }else{
      robotCurrentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four").pose;
    }

    // This is where the hub is located at in meters on the field (from blue alliance).
    Pose2d targetPose = new Pose2d(4.62, 4.05, new Rotation2d(Math.PI));

    // double speedX = m_drivetrain.getState().Speeds.vxMetersPerSecond;
    // double speedY = m_drivetrain.getState().Speeds.vyMetersPerSecond;

    // Pose2d futurePose = poseEstimate.getEstimatedPosition();
    // poseEstimate.update(m_drivetrain.getGyroAngle(), m_drivetrain.getState().ModulePositions);
    
    hasTarget = LimelightHelpers.getTV("limelight-four");
    
    if(hasTarget){
      // Places higher pid values to the turret.
      m_turret.updateValues(3.5, 0.22, 0.8);

      double goalAngle = Math.atan2(targetPose.getY() - robotCurrentPose.getY(), targetPose.getX() - robotCurrentPose.getX());

      // Converts constants into motor rotations for the turret.
      double rotationsError = (goalAngle/360)*TurretConstants.gearRatio;
      double currentRotationsError = (TX/180)*TurretConstants.gearRatio;

      double currentRotation = m_turret.getTurretRotation();

      double currentTargetRotation = currentRotation + currentRotationsError;
      double targetRotation = currentTargetRotation + rotationsError;

      // This is used for testing.
      System.out.println(targetRotation + " | " + rotationsError + " | " + goalAngle + " | " + robotCurrentPose);
      
      if(Math.abs(rotationsError) > .05){
        m_turret.setYaw(targetRotation);
      }
    }

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
