package frc.robot.Commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.TransferFuel;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;

public class ShootingDemo extends SequentialCommandGroup {
    public ShootingDemo(double RPS, Shooter shooter, Conveyor conveyor, Spindexer spindexer) {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new ShootSetRPS(RPS, shooter),
                        new TransferFuel(spindexer, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed)
                    )
                )
            )
        );
    }
}