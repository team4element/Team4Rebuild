/*
 * This command ramps up the shooter before aiming and conveying game pieces.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

public class CombinedShoot extends ParallelCommandGroup {
  /** Creates a new CombinedShoot. */
  public CombinedShoot(Turret turret, Conveyor conveyor, Spinster spinster) {
    addCommands((new ShootParallel(turret, conveyor, spinster).withTimeout(8)).beforeStarting((new ShootForParallel(turret)).withTimeout(0.3)));
  }
}
