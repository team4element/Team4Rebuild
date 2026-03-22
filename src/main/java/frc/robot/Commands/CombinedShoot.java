/*
 * This command ramps up the shooter before aiming and conveying game pieces.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.Shooter;

public class CombinedShoot extends SequentialCommandGroup {
  public CombinedShoot(Shooter shooter, Turret turret, Conveyor conveyor, Spinster spinster) {
    addCommands(
        new ShootForParallel(shooter, turret).withTimeout(0.25), 

        new ParallelCommandGroup(
            new Shoot(shooter, turret), 
            new TransferFuel(spinster, conveyor, -SpinsterConstants.spinsterSpeed, -ConveyorConstants.conveyorSpeed)
        ).withTimeout(9.5)
    );
  }
}

