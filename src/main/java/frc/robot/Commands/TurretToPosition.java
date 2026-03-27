/*
 * This command sets the turret to a desired positon from 0 degrees (home) to it's limits. 
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.Turret;

public class TurretToPosition extends Command {
  /** Creates a new TurretToPosition. */
  Turret m_turret;
  double m_position;

  public TurretToPosition(Turret turret, double position) {
    m_turret = turret;
    m_position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ControllerConstants.operatorController.getLeftX() >= 0.2){
      m_turret.setYaw(-0.1);

    } else if(ControllerConstants.operatorController.getLeftX() < -0.2){
      m_turret.setYaw(0.1);

    } else{
      m_turret.setYaw(m_position);
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
