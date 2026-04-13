/*
 * This command places the pivot of the intake in the desired position. This also runs the outake when the pivot is going up and intake when going down.
 */

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;

public class PositionPivot extends Command {

  private Intake m_intake;
  private Pivot m_pivot;
  private double m_setpoint;
  private double halfIntakeSpeed = IntakeConstants.intakeSpeed/1.5;

  public PositionPivot(Intake intake, Pivot pivot, double setpoint) {
    m_intake = intake;
    m_pivot = pivot;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_setpoint > 10){
      m_intake.runRollers(-halfIntakeSpeed);
    } else{
      m_intake.runRollers(halfIntakeSpeed);
    }
    m_pivot.pivotToSetpoint(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotors();
    m_pivot.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double lowerTolerance = m_setpoint-0.5;
    final double upperTolerance = m_setpoint+0.5;

    final boolean condition = (m_pivot.getPivotPosition() >= lowerTolerance) && (m_pivot.getPivotPosition() <= upperTolerance);

    return condition;
  }
}
