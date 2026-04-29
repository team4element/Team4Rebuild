/*
 * This command moves the pivot for the intake back and forth in order to keep the fuel going through the spindexer.
 */

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;

public class TapPivot extends SequentialCommandGroup {

  private double highSetpoint = 12;
  private double lowSetpoint = 1.5;
  private double timeout = 0.2;

  public TapPivot(Intake intake, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new PositionPivot(intake, pivot, highSetpoint).withTimeout(timeout), new PositionPivot(intake, pivot, lowSetpoint).withTimeout(timeout));
  }
}
