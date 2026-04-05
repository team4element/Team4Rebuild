/*
 * This command manually moves the pivot on the intake between it's limits.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;

public class RetractIntake extends Command {

  private Intake m_intake;
  private Pivot m_pivot;
  private double m_speedPercentagePivot;

  public RetractIntake(Intake intake, Pivot pivot, double speedPercentagePivot) {
    m_intake = intake;
    m_pivot = pivot;
    m_speedPercentagePivot = speedPercentagePivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets the current position of the pivot.
    double currentPivotPosition = m_pivot.getPivotPosition();

    // Checks to see if the pivot is within the physical limits.
    boolean inRange = (currentPivotPosition > PivotConstants.upperPivotLimit) && (currentPivotPosition < PivotConstants.lowerPivotLimit);

    boolean outsideLowRange = (currentPivotPosition <= PivotConstants.upperPivotLimit) && (m_speedPercentagePivot > 0); // RT (down)
    boolean outsideHighRange = (currentPivotPosition >= PivotConstants.lowerPivotLimit) && (m_speedPercentagePivot < 0); // LT (up)

    double halfIntakeSpeed = IntakeConstants.intakeSpeed/2;

    if(inRange || outsideLowRange || outsideHighRange){
      m_pivot.setPivotPercentage(m_speedPercentagePivot);

      if(m_speedPercentagePivot < 0){
        m_intake.runRollers(halfIntakeSpeed);

      }else{
        m_intake.runRollers(-halfIntakeSpeed);
        
      }
    } else {
      m_intake.stopMotors();
    }
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
    return false;
  }
}
