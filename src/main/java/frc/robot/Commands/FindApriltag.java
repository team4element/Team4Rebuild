/*
 * This command is used to track the apriltag with the turret using limelight data. 
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public double targetRotation;
  public double distance;
 // private final double kMinCommand = 0.0003;

  public FindApriltag(Turret turret) {
    m_turret = turret;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Grabs the limelight's data. TX is negative because we want to turn in the opposite direction.
    hasTarget = LimelightHelpers.getTV("limelight-four");
    
    if(hasTarget){
      TX = -LimelightHelpers.getTX("limelight-four");

      // This converts the constant into motor rotations.
      double rotationsError = (TX/360)*TurretConstants.gearRatio;
      double currentRotation = m_turret.getTurretRotation();
      
      // We add the two values so that our goal position isn't just the offest (adds the offset to current position).
      targetRotation = currentRotation + rotationsError;
      
      // This applies the angle of the turret within the deadband/tolerance.
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

    distance = m_turret.shootingDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This puts the data onto the dashboard.
    if(m_turret.getTurretDegree() >= targetRotation-0.5){
      SmartDashboard.putBoolean("Shoot Ready", true);
      m_turret.startShooter(distance);
    }else {
      SmartDashboard.putBoolean("Shoot Ready", false);
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
