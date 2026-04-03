/*
 * This command uses odometry to aim the turret to the hub and vibrates the operator controller when it is lined up within the tolerance.
 */

package frc.robot.Commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;
import frc.robot.Constants.ControllerConstants;

public class AutoAim extends Command {
  private final Turret m_turret;

  public AutoAim(Turret turret) {
    m_turret = turret;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_turret.track();

    if (m_turret.isReadyToShoot()) {
        // Vibrate the operator controller so they know to shoot
        ControllerConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
    } else {
        ControllerConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopMotor(); 

    ControllerConstants.operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // Run until the button is released
  }
}