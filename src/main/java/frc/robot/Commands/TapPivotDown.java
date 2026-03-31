/*
 * This command moves the pivot for the intake back and forth in order to keep the fuel going through the spindexer.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class TapPivotDown extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double initPose;
  public double targetPose;

  public TapPivotDown(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intake.getPivotPosition(m_intake.m_leftPivot) >= 8){

      m_intake.pivotToSetpoint(m_intake.m_leftPivot, 3);
      m_intake.pivotToSetpoint(m_intake.m_rightPivot, 3);

    }
    
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
