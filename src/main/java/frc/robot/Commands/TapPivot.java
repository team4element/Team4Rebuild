/*
 * This command moves the pivot for the intake back and forth in order to keep the fuel going through the spindexer.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;

public class TapPivot extends SequentialCommandGroup {
  /** Creates a new Intake. */
  public Intake m_intake;
  public double v_speedPercentagePivot;

  public TapPivot(Intake intake, double speedPercentagePivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new PositionPivot(intake, 12).withTimeout(0.2), new PositionPivot(intake, 1.5).withTimeout(0.2));
  }
}
