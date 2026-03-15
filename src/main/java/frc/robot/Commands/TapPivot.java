/*
 * This command moves the pivot for the intake back and forth in order to keep the fuel going through the spindexer.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class TapPivot extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double v_speedPercentagePivot;
  public double initPose;

  public TapPivot(Intake intake, double speedPercentagePivot) {
    m_intake = intake;
    v_speedPercentagePivot = speedPercentagePivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = m_intake.getPivotPosition(m_intake.m_leftPivot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.pivotToSetpoint(m_intake.m_leftPivot, initPose+10.5);
    m_intake.pivotToSetpoint(m_intake.m_rightPivot, initPose+10.5);
    
    if(m_intake.getPivotPosition(m_intake.m_leftPivot) >= initPose+7){
      Thread.currentThread();
      try {
        Thread.sleep(300);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      m_intake.pivotToSetpoint(m_intake.m_leftPivot, m_intake.getPivotPosition(m_intake.m_leftPivot)-10.5);
      m_intake.pivotToSetpoint(m_intake.m_rightPivot, m_intake.getPivotPosition(m_intake.m_rightPivot)-10.5);
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
