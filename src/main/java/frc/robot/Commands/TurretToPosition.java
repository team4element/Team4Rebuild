/*
 * This command sets the turret to a desired positon from 0 degrees (home) to a desired motor rotation.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class TurretToPosition extends Command {
  /** Creates a new TurretToPosition. */
  private Turret m_turret;
  private double m_position;

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
    m_turret.setYaw(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
