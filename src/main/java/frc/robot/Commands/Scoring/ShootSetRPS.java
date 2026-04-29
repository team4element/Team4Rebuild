package frc.robot.Commands.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShootSetRPS extends Command {
    private final Shooter m_shooter;
    private final double rps;

    public ShootSetRPS(double RPS, Shooter shooter) {
        rps = RPS;
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
       m_shooter.setRPS(rps);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
