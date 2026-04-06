/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase {
    // Hardware
    private final TalonFX m_motor;

    // Control Requests
    private final DutyCycleOut m_dutyCycleTurret;
    private final PositionVoltage m_positionRequest;
    private final MotionMagicVoltage m_motionMagicRequest;
    private final VelocityVoltage m_velocityRequest;

    // Configurations
    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    // Dependencies
    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagFieldLayout m_field_layout;

    // Logic State
    private double lastP, lastD, lastS, lastPS, lastDS, lastVS;

    // AdvantageScope Data
    private final StructPublisher<Pose2d> turretPosePublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/FieldPose", Pose2d.struct).publish();

    private final StructPublisher<Translation2d> targetPosePublisher =
    NetworkTableInstance.getDefault().getStructTopic("Turret/TargetPose", Translation2d.struct).publish();

    public Turret(AprilTagFieldLayout field_layout, CommandSwerveDrivetrain drivetrain) {
        m_motor = new TalonFX(TurretConstants.turretID);
        m_field_layout = field_layout;
        m_drivetrain = drivetrain;

        m_dutyCycleTurret = new DutyCycleOut(TurretConstants.dutyCycleTurret);
        m_positionRequest = new PositionVoltage(0).withSlot(0);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0);
        m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

        // --- Turret Config ---
        turretConfig.Slot0.kP = TurretConstants.KPTurret;
        turretConfig.Slot0.kI = 0;
        turretConfig.Slot0.kD = TurretConstants.KDTurret;
        turretConfig.Slot0.kS = TurretConstants.KSTurret;

        turretConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turretMaxVelocity;
        turretConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turretMaxAcceleration;
        turretConfig.MotionMagic.MotionMagicJerk = TurretConstants.turretMaxJerk;

        turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.turretStatorLimit;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.turretSupplyLimit;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motor.getConfigurator().apply(turretConfig);

        lastP = TurretConstants.KPTurret;
        lastD = TurretConstants.KDTurret;
        lastS = TurretConstants.KSTurret;

        // --- Put Data on the Dashboard ---
        SmartDashboard.putNumber("Turret kP", lastP);

        // --- Vision Configs ---
        LimelightHelpers.SetIMUMode("limelight-four", 0);
    }

    /*
     * Sets the turret's starting position (homing).
     */
    public void resetTurret(){
        m_motor.setPosition(0);
    }

    /**
     * Assigns a speed to run the turret motor using PID.
     * @param RPS from 0 to 200.
     */
    public void spinTurret(double RPS){
        m_motor.setControl(m_velocityRequest.withVelocity(RPS).withSlot(0));
    }

    /**
     * Assigns power to the turret motor based on a percentage.
     * @param percentage from -1 to 1.
     */
    public void setTurretPercentage(double percentage){
        m_motor.setControl(m_dutyCycleTurret.withOutput(percentage));
    }

    /**
     * Powers the turret motor through a position in rotations.
     * @param angle to turn to.
     */
    public void setYaw(double angle) {
        m_motor.setControl(m_positionRequest.withPosition(angle*TurretConstants.gearRatio));
    }

    /*
     * Stops the turret movement.
     */
    public void stopMotor(){
        m_motor.setControl(m_dutyCycleTurret.withOutput(0));
    }

    /**
     * Grabs the turret motor's rotation and converts it into the turret's position by dividing with the gear ratio.
     * @return degree of turret.
     */
    public double getTurretDegree(){
        double motorRotations = m_motor.getPosition().getValueAsDouble();
        double turretRotations = motorRotations / TurretConstants.gearRatio;
        double turretDegrees = turretRotations * 360.0;

        return turretDegrees;
    }

    /**
     * Grabs the turret motor's rotation and converts it into the turret's position by dividing with the gear ratio.
     * @return the motor rotation of turret.
     */
    public double getTurretRotation(){
        return m_motor.getPosition().getValueAsDouble();
    }

    /**
     * Allows the operator to control the direction of the turret.
     * Stops the motor once the turret reaches the left or right limits.
     */
    public void rotateManual(){
        if(ControllerConstants.operatorController.povRight().getAsBoolean()){
             double limit = TurretConstants.leftLimit;

            if(getTurretDegree() <= limit){
                setTurretPercentage(-0.1);

            }else if(getTurretDegree() >= limit){
               stopMotor();

            }
        }else if(ControllerConstants.operatorController.povLeft().getAsBoolean()){
             double limit = TurretConstants.rightLimit;

            if(getTurretDegree() <= limit){
                stopMotor();

            }else if(getTurretDegree() >= limit){
                setTurretPercentage(0.1);

            }
        }else{
            stopMotor();
        }
    }

    /**
     * Updates the PID values of the turret motor.
     * @param P
     * @param D
     * @param S
     */
    public void updateValues(double P, double D, double S, double PS, double DS, double VS){
        if(P != lastP || D != lastD || S != lastS || PS != lastPS || DS != lastDS || VS != lastVS){
            turretConfig.Slot0.kP = P;
            turretConfig.Slot0.kD = D;
            turretConfig.Slot0.kS = S;
            m_motor.getConfigurator().apply(turretConfig);

            lastP = P; lastD = D; lastS = S; lastPS = PS; lastDS = DS; lastVS = VS;
        }
    }

    /**
     * Uses the physical limits to ensure that the target rotation keeps within bounds.
     * @param targetRotations for the turret motor.
     * @return the clamped value.
     */
    private double clampTurretRotations(double targetRotations) {
        double leftLimitRot = (TurretConstants.leftLimit / 360.0) * TurretConstants.gearRatio;
        double rightLimitRot = (TurretConstants.rightLimit / 360.0) * TurretConstants.gearRatio;
        return MathUtil.clamp(targetRotations, rightLimitRot, leftLimitRot);
    }

    /**
     * Uses the target angle to tell the motor where to turn depending on it's current position.
     * This makes sure that the turret keeps within it's physical limits.
     * @param targetAngle for the turret to turn to.
     * @return motor rotations. 
     */
    public double calculateSmartWrap(Rotation2d targetAngle) {
        double currentMotorRotations = m_motor.getPosition().getValueAsDouble();
        double currentMotorDegrees = (currentMotorRotations / TurretConstants.gearRatio) * 360.0;

        double targetDeg = targetAngle.getDegrees();
        double delta = Math.IEEEremainder(targetDeg - currentMotorDegrees, 360.0);
        double closestTarget = currentMotorDegrees + delta;

        // Verify bounds
        if (closestTarget < TurretConstants.rightLimit || closestTarget > TurretConstants.leftLimit) {
            double altTarget = (closestTarget > currentMotorDegrees) ? closestTarget - 360 : closestTarget + 360;

            if (altTarget >= TurretConstants.rightLimit && altTarget <= TurretConstants.leftLimit) {
                closestTarget = altTarget;
            } else {
                closestTarget = MathUtil.clamp(closestTarget, TurretConstants.rightLimit, TurretConstants.leftLimit);
            }
        }

        return (closestTarget / 360.0) * TurretConstants.gearRatio;
    }

    /*
     * Calculates where the turret should aim using odometry. 
     * This calculates the turret's position relative to the robot and finds the error angle to the hub. 
     * Uses the error as the target angle and uses clamping to make sure it will move within physical limits.
     * Updates the position of the turret on the field by subtracting the current position (relative to the robot) by the hub position.
     */
    public void track() {
        // Identify Target based on Alliance
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int targetTagID = (alliance == Alliance.Blue) ? VisionConstants.centerHubBlueTag : VisionConstants.centerHubRedTag;

        // Gets the pose of the hub on the field based on the center tag.
        var hubPose = m_field_layout.getTagPose(targetTagID);
        if (!hubPose.isEmpty()) {
            Translation2d hubCenter = hubPose.get().toPose2d().getTranslation();
            
            double centerOffsetMeters = 0.6; // Adjust this based on your specific goal depth
            double xOffset = (alliance == Alliance.Blue) ? centerOffsetMeters : -centerOffsetMeters;

            Translation2d hubCenterLocation = new Translation2d(
                hubCenter.getX() + xOffset,
                hubCenter.getY() // Keep Y the same if the tag is centered on the goal
            );

            // Get Current Robot State from CTRE Swerve
            Pose2d robotPose = m_drivetrain.getState().Pose;

            // Calculate where the TURRET pivot is on the field (Top-Right mount)
            // Robot center is (0,0). Positive X is forward, Negative Y is right.
            Transform2d robotToTurret = new Transform2d(
                new Translation2d(TurretConstants.robotCenterToTurretForward, -TurretConstants.robotCenterToTurretRight),
                new Rotation2d() // Turret base is fixed to the robot grid
            );

            Pose2d turretPose = robotPose.transformBy(robotToTurret);

            // Calculate target angle from the TURRET PIVOT to the Hub
            Rotation2d fieldAngleFromTurret = hubCenterLocation.minus(turretPose.getTranslation()).getAngle();

            // Calculate how much the turret needs to rotate relative to the robot chassis
            Rotation2d turretTargetRelative = fieldAngleFromTurret.minus(robotPose.getRotation());

            // Calculate the motor setpoint
            double finalMotorSetpoint = calculateSmartWrap(turretTargetRelative);

            // Enforce hardware rotation limits
            double safeSetpoint = clampTurretRotations(finalMotorSetpoint);

            // Send command to the motor using the Motion Magic profile defined in constants
            //TODO: Test motion magic vs position voltage;
            m_motor.setControl(m_motionMagicRequest.withPosition(safeSetpoint));
            // m_motor.setControl(m_positionRequest.withPosition(safeSetpoint));

            //--------------------- ADVANTAGESCOPE TELEMETRY

            // Get the turret's current relative angle using your clean degrees method
            double currentDegrees = getTurretDegree();
            Rotation2d currentRelativeAngle = Rotation2d.fromDegrees(currentDegrees);

            // Add the robot's current rotation to get the Turret's GLOBAL rotation on the field
            Rotation2d globalTurretAngle = robotPose.getRotation().plus(currentRelativeAngle);

            // Construct the final global Pose2d for AdvantageScope
            Pose2d actualTurretPose = new Pose2d(turretPose.getTranslation(), globalTurretAngle);

            // Publish to NetworkTables
            turretPosePublisher.set(actualTurretPose);
            // targetPosePublisher.set(hubCenterLocation);
        }
    }

    // Determines if the turret is lined up to shoot fuel.
    public boolean isReadyToShoot() {
	    //TODO: Update me
        return false;
    }
}
