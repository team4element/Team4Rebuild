/*
 * This command ramps up the shooter before conveying game pieces and shooting at a set speed.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Shooter;

public class CornerShot extends SequentialCommandGroup {
  public CornerShot(Shooter shooter, Conveyor conveyor, Spindexer spinster, double RPS) {
    addCommands(
        new ShootWithoutEnd(shooter).withTimeout(0.25), 

        new ParallelCommandGroup(
            new Shoot(shooter, RPS), 
            new TransferFuel(spinster, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed)
        ).withTimeout(9.5)
    );
  }
}

