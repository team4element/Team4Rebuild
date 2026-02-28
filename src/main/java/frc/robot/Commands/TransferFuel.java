package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Spinster;

public class TransferFuel extends ParallelCommandGroup {
  /** Creates a new TransferFuel. */

  public TransferFuel(Spinster spinster, Conveyor conveyor, double speedPercentage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new GumballRotation(spinster).withTimeout(2).alongWith(new ConveyToTurret(conveyor, speedPercentage)).withTimeout(2));
  }
}
