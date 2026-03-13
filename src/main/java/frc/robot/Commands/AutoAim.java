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
  public void initialize() {
    // Optional: Set a specific Limelight pipeline for tracking
    // LimelightHelpers.setPipelineIndex("limelight-four", 0);
  }

  @Override
  public void execute() {
    m_turret.trackAndShoot();

    if (m_turret.isReadyToShoot()) {
        // Vibrate the operator controller so they know to hit the "Fire" button
        ControllerConstants.operatorController.getHID()
            .setRumble(RumbleType.kBothRumble, 0.2);
    } else {
        ControllerConstants.operatorController.getHID()
            .setRumble(RumbleType.kBothRumble, 0);
    }
}

  @Override
  public void end(boolean interrupted) {
    m_turret.stopMotors(); 

    ControllerConstants.operatorController.getHID()
            .setRumble(RumbleType.kBothRumble, 0);
    // Or: m_turret.returnToStartPosition();
  }

  @Override
  public boolean isFinished() {
    return false; // Run until the button is released
  }
}