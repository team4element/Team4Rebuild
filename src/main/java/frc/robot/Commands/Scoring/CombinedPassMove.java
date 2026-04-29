package frc.robot.Commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.TransferFuel;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Turret;

public class CombinedPassMove extends SequentialCommandGroup {
    public CombinedPassMove(Turret turret, Shooter shooter, Conveyor conveyor, Spindexer spindexer, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new ParallelCommandGroup(
               new AutoAimPass(turret),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                        new PassMove(shooter, turret, drivetrain),
                        new TransferFuel(spindexer, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed)
                    )
                )
            )
        );
    }
}