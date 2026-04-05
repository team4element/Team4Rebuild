package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class AutoAimMove extends Command {
    private final Turret m_turret;

    public AutoAimMove(Turret turret, AprilTagFieldLayout fieldLayout) {
        m_turret = turret;
        // We don't even need m_fieldLayout in here anymore since trackVirtualTarget handles it!

        // ONLY require the turret so it can run independently of the shooter
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        // This single line does all the math and moves the turret to lead the moving target!
        m_turret.trackVirtualTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Safely stop the turret motor when the command ends
        m_turret.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}