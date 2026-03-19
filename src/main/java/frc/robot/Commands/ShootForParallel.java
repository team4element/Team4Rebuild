/*
 * This command runs the shooter with no end. 
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;

public class ShootForParallel extends Command {
  /** Creates a new Shoot. */
  public Turret m_turret;
  public Shooter m_shooter;
  
  public ShootForParallel(Shooter shooter, Turret turret) {
    m_turret = turret;
    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setRPS(m_turret.shootingDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
