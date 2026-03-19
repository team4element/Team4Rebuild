/*
 * This command runs the shooter with no end. 
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShootForParallelAuto extends Command {
  /** Creates a new Shoot. */
  public Shooter m_shooter;
  public double m_RPS;
  
  public ShootForParallelAuto(Shooter shooter, double RPS) {
    m_shooter = shooter;
    m_RPS = RPS;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setRPS(m_RPS);
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
