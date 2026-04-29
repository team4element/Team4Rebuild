/*
 * This command controls the pivot without the physical limits.
 */

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;

public class FreePivot extends Command {

  private Pivot m_pivot;
  private Intake m_intake;
  private double m_speedPercentage;

  public FreePivot(Pivot pivot, Intake intake, double speedPercentage) {
    m_pivot = pivot;
    m_intake = intake;
    m_speedPercentage = speedPercentage;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setPivotPercentage(m_speedPercentage);

    double halfIntakeSpeed = IntakeConstants.intakeSpeed/2;

    if(m_speedPercentage < 0){
        m_intake.runRollers(halfIntakeSpeed);

      }else{
        m_intake.runRollers(-halfIntakeSpeed);
        
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stopMotors();
    m_intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
