/*
 * This command ramps up the shooter before aiming and conveying game pieces.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Shooter;

public class CombinedTapShoot extends SequentialCommandGroup {
  public CombinedTapShoot(Shooter shooter, Conveyor conveyor, Spinster spinster, Intake intake, Pivot pivot) {
    addCommands(
        new ShootWithoutEnd(shooter).withTimeout(0.25), 

        new ParallelCommandGroup(
            new Shoot(shooter), 
            new TransferFuel(spinster, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed),
            new TapPivot(intake, pivot).repeatedly().withTimeout(3)
        ).withTimeout(9.5)
    );
  }
}

