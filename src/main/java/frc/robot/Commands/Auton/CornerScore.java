// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.CombinedTapShootCorner;
import frc.robot.Commands.TurretToPosition;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CornerScore extends ParallelCommandGroup {
  /** Creates a new CornerScore. */
  public CornerScore(Shooter shooter, Turret turret, Conveyor conveyor, Spinster spinster, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands( new CombinedTapShootCorner(shooter, conveyor, spinster, intake), new TurretToPosition(turret, 0));
  }
}
