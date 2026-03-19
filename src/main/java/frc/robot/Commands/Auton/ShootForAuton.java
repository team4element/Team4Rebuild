/*
 * This command ramps up the shooter before aiming and conveying game pieces.
 */

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

public class ShootForAuton extends ParallelCommandGroup {
  /** Creates a new CombinedShoot. */
  public ShootForAuton(Shooter shooter, Turret turret, Conveyor conveyor, Spinster spinster, double RPS) {
    addCommands((new ShootParallelAuto(shooter, turret, conveyor, spinster, RPS).withTimeout(10)).beforeStarting((new ShootForParallelAuto(shooter, RPS)).withTimeout(0.3)));
  }
}
