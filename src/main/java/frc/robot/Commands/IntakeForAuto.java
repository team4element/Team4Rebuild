/*
 * This command is used for auto to bring the pivot of the intake down and intake the fuel. 
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Auton.PivotForAuto;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;

public class IntakeForAuto extends SequentialCommandGroup {
  /** Creates a new IntakeForAuto. */
  public IntakeForAuto(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new PivotForAuto(intake).withTimeout(IntakeConstants.intakeTimeout), new IntakeFuel(intake, IntakeConstants.intakeSpeed).withTimeout(IntakeConstants.intakeTimeout));
  }
}
