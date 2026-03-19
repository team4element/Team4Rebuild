/*
 * This command ramps up the shooter before aiming and conveying game pieces.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

public class CombinedShoot extends SequentialCommandGroup {
  public CombinedShoot(Turret turret, Conveyor conveyor, Spinster spinster) {
    addCommands(
        new ShootForParallel(turret).withTimeout(0.5), 

        new ParallelCommandGroup(
            new Shoot(turret), 
            new TransferFuel(spinster, conveyor, -SpinsterConstants.spinsterSpeed, -ConveyorConstants.conveyorSpeed)
        ).withTimeout(9.5)
    );
  }
}

