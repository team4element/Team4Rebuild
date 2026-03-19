/*
 * This command is used to track, shoot, and transfer game pieces in an automated fashion (running at the same time).
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.TransferFuel;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

public class ShootParallelAuto extends ParallelCommandGroup {
  /** Creates a new CombinedShoot. */
  public ShootParallelAuto(Shooter shooter, Turret turret, Conveyor conveyor, Spinster spinster, double RPS) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new TransferFuel(spinster, conveyor, 0.75, -1), new Shoot(shooter, turret, RPS));
  }
}
