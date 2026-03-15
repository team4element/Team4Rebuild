/*
 * This command runs the spindexer and conveyor at the same time in order to pass the balls through to the shooter. s
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spinster;

public class TransferFuel extends ParallelCommandGroup {
  /** Creates a new TransferFuel. */

  public TransferFuel(Spinster spinster, Conveyor conveyor, double speedPercentageConvey, double speedPercentageSpin) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new GumballRotation(spinster, speedPercentageSpin), (new ConveyToTurret(conveyor, speedPercentageConvey)));
  }
}
