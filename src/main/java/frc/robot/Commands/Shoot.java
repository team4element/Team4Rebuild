/*
 * This command runs the shooter at the RPS measured using the regression formula and a calculated distance.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  public Turret m_turret;
  private final double let_distance_decide = -1;
  private double m_RPS;

  public Shoot(Turret turret, double RPS) {
    m_turret = turret;
    m_RPS = RPS;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  public Shoot(Turret turret) {
      m_turret = turret;
      m_RPS = let_distance_decide;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // distance = m_turret.shootingDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint;

    if (m_RPS == let_distance_decide) {
        setpoint = m_turret.shootingDistance();
    } else {
        setpoint = m_RPS; // This will stay 50 every single loop
    }

    m_turret.startShooter(setpoint);
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
