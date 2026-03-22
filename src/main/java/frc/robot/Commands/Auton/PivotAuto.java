/*
 * This command places the pivot of the intake in the desired position (limits) depending on where it was last. So if the pivot was near 0, it will 
 * go to the higher limit and vice versa.
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;

public class PivotAuto extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double m_setpoint;

  public PivotAuto(Intake intake, double setpoint) {
    m_intake = intake;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.automaticPivot(m_intake.m_leftPivot, m_setpoint-2);
    m_intake.automaticPivot(m_intake.m_rightPivot, m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runRollers(IntakeConstants.intakeSpeed);
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
