package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;

public class ConveyToTurret extends Command {
  /** Creates a new TransferFuel. */
  public Conveyor m_conveyor;

  public double v_speedPercentage;
  
  public ConveyToTurret(Conveyor conveyor, double speedPercentage) {
    m_conveyor = conveyor;
     
    v_speedPercentage = speedPercentage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.runMotor(v_speedPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
