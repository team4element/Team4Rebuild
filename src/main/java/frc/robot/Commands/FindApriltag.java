package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.Turret;

public class FindApriltag extends Command {
  /** Creates a new FollowApriltag. */
  public Turret m_turret;
  public double TX;
  public boolean hasTarget;
  public double currentPose;
  public double FPS;
 // private final double kMinCommand = 0.0003;

  public FindApriltag(Turret turret) {
    m_turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TX = -LimelightHelpers.getTX("limelight-four");
    hasTarget = LimelightHelpers.getTV("limelight-four");
    
    if(hasTarget){
      // This converts the constant into motor rotations.
      double rotationsError = (TX/360)*TurretConstants.gearRatio;
      double currentRotation = m_turret.getTurretRotation();
      
      double targetRotation = currentRotation + rotationsError;
      
      if(Math.abs(rotationsError) > .05){
        m_turret.setYaw(targetRotation);
      }

    } else{
      // This is used to give the turret enough power to 'scan' the area within it's physical limits until it sees an apriltag.
      double searchSpeed;

      if(m_turret.getTurretDegree() > 0 || m_turret.getTurretDegree() > TurretConstants.leftLimit){
        searchSpeed = -0.05;
      }
      else if(m_turret.getTurretDegree() < 0 || m_turret.getTurretDegree() < TurretConstants.rightLimit){
        searchSpeed = 0.05;

      } else{
        searchSpeed = 0;
      }

      m_turret.setTurretPercentage(searchSpeed);
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
