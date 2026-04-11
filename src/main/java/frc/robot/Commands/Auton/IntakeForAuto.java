/*
 * This command is used for auto to bring the pivot of the intake down and intake the fuel. 
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.IntakeFuel;
import frc.robot.Commands.PositionPivot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;

public class IntakeForAuto extends SequentialCommandGroup {
  /** Creates a new IntakeForAuto. */
  public IntakeForAuto(Intake intake, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new WaitCommand(1.3), new PositionPivot(intake, pivot, 18.6), new IntakeFuel(intake, IntakeConstants.intakeSpeed));
  }
}
