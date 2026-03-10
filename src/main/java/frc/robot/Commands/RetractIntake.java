package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class RetractIntake extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double v_speedPercentagePivot;

  public RetractIntake(Intake intake, double speedPercentagePivot) {
    m_intake = intake;
    v_speedPercentagePivot = speedPercentagePivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.extendIntake(m_intake.m_leftPivot, v_speedPercentagePivot);
    m_intake.extendIntake(m_intake.m_rightPivot, v_speedPercentagePivot);
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
