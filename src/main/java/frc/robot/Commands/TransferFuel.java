/*
 * This command runs the spindexer and conveyor at the same time in order to pass the balls through to the shooter.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spindexer;

public class TransferFuel extends ParallelCommandGroup {

  public TransferFuel(Spindexer spinster, Conveyor conveyor, double speedPercentageSpin, double speedPercentageConvey) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new GumballRotation(spinster, speedPercentageSpin), (new ConveyToTurret(conveyor, speedPercentageConvey)));
  }
}
