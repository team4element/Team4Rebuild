/*
 * This command ramps up the shooter before conveying game pieces and shooting while tapping the pivot.
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.ShootWithoutEnd;
import frc.robot.Commands.TapPivot;
import frc.robot.Commands.TransferFuel;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Shooter;

public class CombinedTapShootAuto extends SequentialCommandGroup {
  public CombinedTapShootAuto(Shooter shooter, Conveyor conveyor, Spindexer spinster, Intake intake, Pivot pivot) {
    addCommands(
        new ShootWithoutEnd(shooter).withTimeout(0.25), 

        new ParallelCommandGroup(
            new Shoot(shooter), 
            new TransferFuel(spinster, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed),
            new TapPivot(intake, pivot).repeatedly().withTimeout(8)
        ).withTimeout(9.5)
    );
  }
}

