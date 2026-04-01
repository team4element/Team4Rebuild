/*
 * This command runs the shooter at the RPS measured using the regression formula and a calculated distance.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  private Shooter m_shooter;
  private final double let_distance_decide = -1;
  private double m_RPS;

  public Shoot(Shooter shooter, double RPS) {
    m_shooter = shooter;
    m_RPS = RPS;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  public Shoot(Shooter shooter) {
      m_shooter = shooter;
      m_RPS = let_distance_decide;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint;

    if (m_RPS == let_distance_decide) {
        setpoint = m_shooter.shootingDistance();
    } else {
        setpoint = m_RPS; // This will stay 50 every single loop
    }

    m_shooter.setRPS(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
