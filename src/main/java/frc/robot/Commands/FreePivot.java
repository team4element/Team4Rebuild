// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FreePivot extends Command {
  /** Creates a new FreePivot. */
  private Pivot m_pivot;
  private double m_speedPercentage;

  public FreePivot(Pivot pivot, double speedPercentage) {
    m_pivot = pivot;
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
