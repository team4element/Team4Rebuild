/*
 * This command is used to track, shoot, and transfer game pieces in an automated fashion (running at the same time).
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

public class ShootParallel extends ParallelCommandGroup {
  /** Creates a new CombinedShoot. */
  public ShootParallel(Turret turret, Conveyor conveyor, Spinster spinster) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new TransferFuel(spinster, conveyor, SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed), new Shoot(turret));
  }
}
