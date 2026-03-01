package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;

public class IntakeAndRetract extends SequentialCommandGroup {
  /** Creates a new IntakeAndRetract. */

  public IntakeAndRetract(Intake intake, double speedPercentageSlide, double speedPercentageIntake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new IntakeFuel(intake, speedPercentageSlide, speedPercentageIntake).andThen(new RetractIntake(intake, speedPercentageIntake)));
  }
}
