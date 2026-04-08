package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Turret;

public class VisionAlignAndZero extends Command {
    private final Turret m_turret;
    private final Timer m_timeout = new Timer();
    private int m_stableFrames = 0;

    public VisionAlignAndZero(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        m_stableFrames = 0;
        m_timeout.reset();
        m_timeout.start();
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(VisionConstants.kLimelightName)) {
         
            double tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);
            
            double currentMotorRotations = m_turret.getTurretRotation();
            double errorRotations = (tx / 360.0) * TurretConstants.gearRatio;
            double targetRotations = currentMotorRotations + errorRotations;

            m_turret.setYaw(targetRotations / TurretConstants.gearRatio); 

            if (Math.abs(tx) < 0.5) {
                m_stableFrames++;
            } else {
                m_stableFrames = 0;
            }
        } else {
            m_turret.stopMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return m_stableFrames >= 10 || m_timeout.hasElapsed(3.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stopMotor();

        if (!interrupted && m_stableFrames >= 10) {
            m_turret.resetTurret();
            System.out.println("TURRET ALIGNMENT FIXED: New Zero point set.");
        }
    }
}