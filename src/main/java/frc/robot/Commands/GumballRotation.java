/*
 * This command is used to run the conveyor. 
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Spinster;

public class GumballRotation extends Command {
  /** Creates a new TransferFuel. */
  public Spinster m_spinster;
  public double m_percentage;
  
  public GumballRotation(Spinster spinster, double percentage) {
    m_spinster = spinster;
    m_percentage = percentage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinster);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spinster.runMotor(m_percentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinster.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
