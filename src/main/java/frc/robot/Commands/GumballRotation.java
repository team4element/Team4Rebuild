package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Spinster;

public class GumballRotation extends Command {
  /** Creates a new TransferFuel. */
  public Spinster m_spinster;
  
  public GumballRotation(Spinster spinster) {
    m_spinster = spinster;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinster);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spinster.rotateSlot();
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
