package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class AutoAimMove extends Command {
    private final Turret m_turret;

    public AutoAimMove(Turret turret, AprilTagFieldLayout fieldLayout) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.trackVirtualTarget();
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
