package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Turret;

public class FindApriltag extends Command {
  /** Creates a new FollowApriltag. */
  public Turret m_turret;
  public double TX;
  public boolean hasTarget;
  public double currentPose;
  private final double kMinCommand = 0.0003;

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
      final double gearRatio = 7.01;
      double rotationsError = (TX/360)*gearRatio;
      double currentRotation = m_turret.getTurretRotation();
      
      double targetRotation = currentRotation + rotationsError;

      System.out.println(targetRotation + " | " + rotationsError + " | " + TX + " | " + currentRotation);
      
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
