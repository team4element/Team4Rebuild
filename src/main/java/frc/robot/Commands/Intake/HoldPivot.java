package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot;

public class HoldPivot extends Command {
  /** Creates a new HoldPivot. */
  Pivot m_pivot;

  public HoldPivot(Pivot pivot) {
    m_pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_pivot.hold();
    m_pivot.setPivotPercentage(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double lowerTolerance = 0-0.5;
    final double upperTolerance = 0+0.5;

    final boolean condition = (m_pivot.getPivotPosition() >= lowerTolerance) && (m_pivot.getPivotPosition() <= upperTolerance);

    return condition;
  }
}
