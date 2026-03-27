/*
 * This command moves the pivot for the intake back and forth in order to keep the fuel going through the spindexer.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class TapPivot extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double initPose;
  public double targetPose;

  public TapPivot(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initPose = m_intake.getPivotPosition(m_intake.m_leftPivot);
    targetPose = initPose + 5.5;
    m_intake.pivotToSetpoint(m_intake.m_leftPivot, targetPose-2);
    m_intake.pivotToSetpoint(m_intake.m_rightPivot, targetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intake.getPivotPosition(m_intake.m_leftPivot) >= initPose+2){
      targetPose = initPose - 9.5;
      Thread.currentThread();

      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      m_intake.pivotToSetpoint(m_intake.m_leftPivot, targetPose-2);
      m_intake.pivotToSetpoint(m_intake.m_rightPivot, targetPose);
    } else if(m_intake.getPivotPosition(m_intake.m_leftPivot) >= initPose-5){
      targetPose = initPose + 9.5;
      Thread.currentThread();

      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      
      m_intake.pivotToSetpoint(m_intake.m_leftPivot, targetPose-2);
      m_intake.pivotToSetpoint(m_intake.m_rightPivot, targetPose);
    }

    System.out.println("target pose: " + targetPose);
    System.out.println("current pose: "+ m_intake.getPivotPosition(m_intake.m_leftPivot));
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
