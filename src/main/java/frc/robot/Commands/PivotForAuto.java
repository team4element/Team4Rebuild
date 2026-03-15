/*
 * This command is meant to move the pivot of the intake for auto (a little higher than the limit).
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class PivotForAuto extends Command {
  /** Creates a new PivotForAuto. */
  Intake m_intake;

  public PivotForAuto(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.pivotToSetpoint(m_intake.m_leftPivot, 16);
    m_intake.pivotToSetpoint(m_intake.m_rightPivot, 16);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
