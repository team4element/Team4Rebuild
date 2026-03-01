package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeFuel extends Command {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double v_speedPercentageSlide;
  public double v_speedPercentageIntake;

  public IntakeFuel(Intake intake, double speedPercentageSlide, double speedPercentageIntake) {
    m_intake = intake;
    v_speedPercentageSlide = speedPercentageSlide;
    v_speedPercentageIntake = speedPercentageIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.extendLinearSlideManual(v_speedPercentageSlide);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runRollers(v_speedPercentageIntake);
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
