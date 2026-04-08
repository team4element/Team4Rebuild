package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Turret;

public class AutoAimPass extends Command {
    private final Turret m_turret;

    public AutoAimPass(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void execute() {
        m_turret.trackPassTarget();
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