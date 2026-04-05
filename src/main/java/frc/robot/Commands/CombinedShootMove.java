package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.SpinsterConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Turret;

public class CombinedShootMove extends ParallelCommandGroup {

  public CombinedShootMove(Turret turret, Shooter shooter, Conveyor conveyor, Spindexer spindexer, CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout fieldLayout) {
    
    addCommands(
        new ShootMove(shooter, turret, drivetrain, fieldLayout),
        new TransferFuel(spindexer, conveyor, -SpinsterConstants.spinsterSpeed, ConveyorConstants.conveyorSpeed)
    );
  }
}