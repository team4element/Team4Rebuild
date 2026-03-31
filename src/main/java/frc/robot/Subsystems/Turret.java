/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase {
    // Hardware
    private final TalonFX m_turret;

    // Control Requests
    private final DutyCycleOut m_dutyCycleTurret;
    private final PositionVoltage m_positionRequest;
    private final VelocityVoltage m_velocityRequest; 

    // Configurations
    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    // Dependencies
    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagFieldLayout m_field_layout;

    // Logic State
    private double lastP, lastD, lastS, lastPS, lastDS, lastVS;
    private int m_visionLostCounter = 0;
    private int kVisionThreshold = 5;

    // ETC
    Translation2d virtualHubLocation = new Translation2d(0, 0);

    Pose2d robotPose; 
    PoseEstimate mt1;

    StructPublisher<Pose2d> publisher;
    StructPublisher<Pose2d> limeLightPublisher;

    @SuppressWarnings("unused")
    private boolean debug = false;

    public Turret(AprilTagFieldLayout field_layout, CommandSwerveDrivetrain drivetrain) {
        publisher          = NetworkTableInstance.getDefault().getStructTopic("botPose", Pose2d.struct).publish();
        limeLightPublisher = NetworkTableInstance.getDefault().getStructTopic("limelightPose", Pose2d.struct).publish();

        m_turret = new TalonFX(TurretConstants.turretID);
        m_field_layout = field_layout;
        m_drivetrain = drivetrain;

        robotPose = m_drivetrain.getState().Pose;

        m_dutyCycleTurret = new DutyCycleOut(TurretConstants.dutyCycleTurret);
        m_positionRequest = new PositionVoltage(0).withSlot(0);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0);

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

        m_turret.getConfigurator().apply(turretConfig);

        lastP = TurretConstants.KPTurret;
        lastD = TurretConstants.KDTurret;
        lastS = TurretConstants.KSTurret;

        SmartDashboard.putNumber("Turret kP", lastP);
    
        // --- Vision Configs ---
        m_visionLostCounter = 0;
        LimelightHelpers.SetIMUMode("limelight-four", 0);
    }

    /*
     * Sets the turret's starting position. (Homing)
     */
    public void resetTurret(){
        m_turret.setPosition(0);
    }

    /**
     * Assigns a speed to run the turret motor using PID.
     * @param RPS from 0 to 200.
     */
    public void spinTurret(double RPS){
        m_turret.setControl(m_velocityRequest.withVelocity(RPS).withSlot(0));
    }

    /**
     * Assigns power to the turret motor based on a percentage.
     * @param percentage from -1 to 1.
     */
    public void setTurretPercentage(double percentage){
        m_turret.setControl(m_dutyCycleTurret.withOutput(percentage));
    }

    /**
     * Powers the turret motor through a position in rotations.
     * @param angle between limits.
     */
    public void setYaw(double angle) {
        m_turret.setControl(m_positionRequest.withPosition(angle*TurretConstants.gearRatio));
    }

    /*
     * Stops both the turret  movement.
     */
    public void stopMotors(){
        m_turret.setControl(m_dutyCycleTurret.withOutput(0));
    }

    /*
     * Moves the turret to it's 0 position (facing forward).
     */
    public void returnToStartPosition(){
        m_turret.setControl(m_positionRequest.withPosition(0));
    }

    /**
     * Grabs the turret motor's rotation and converts it into the turret's position by dividing with the gear ratio.
     * @return degree of turret.
     */
    public double getTurretDegree(){
        double motorRotations = m_turret.getPosition().getValueAsDouble();
        double turretRotations = motorRotations / TurretConstants.gearRatio;
        double turretDegrees = turretRotations * 360.0;

        return turretDegrees;
    }

    /**
     * Grabs the turret motor's rotation and converts it into the turret's position by dividing with the gear ratio.
     * @return the motor rotation of turret.
     */
    public double getTurretRotation(){
        return m_turret.getPosition().getValueAsDouble();
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

                m_turret.set(0);

            }
        }else if(ControllerConstants.operatorController.povLeft().getAsBoolean()){
             double limit = TurretConstants.rightLimit;

            if(getTurretDegree() <= limit){
                m_turret.set(0);

            }else if(getTurretDegree() >= limit){
                setTurretPercentage(0.1);

            }
        }else{
            m_turret.set(0);
        }
    }

    /**
    * ONE function to rule them all. 
    * Returns distance in METERS using Vision (primary) or Odometry (fallback).
    */
    public double getBestDistanceMeters() {
        boolean seeTag = LimelightHelpers.getTV("limelight-four");
        double distMeters = 0;

        // 1. Update Flicker Protection
        if (seeTag) {
            m_visionLostCounter = 0;
        } else {
            m_visionLostCounter++;
        }

        // 2. Determine Distance Source
        if (seeTag && m_visionLostCounter < kVisionThreshold) {
            // --- VISION STRATEGY ---
            double[] pose = NetworkTableInstance.getDefault()
                .getTable("limelight-four")
                .getEntry("targetpose_robotspace")
                .getDoubleArray(new double[0]);

            if (pose.length >= 6) {
                // Targetpose_robotspace: [x, y, z, roll, pitch, yaw]
                double x = pose[0];
                double z = pose[2];
                distMeters = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
            } else {
                distMeters = getOdometryDistanceMeters();
            }
        } else {
            // --- ODOMETRY STRATEGY ---
            distMeters = getOdometryDistanceMeters();
        }

        // 3. Final Validation
        if (distMeters > TurretConstants.distanceUpperLimit || distMeters < TurretConstants.distanceLowerLimit) {
            // If out of physical range, default to Odometry as a safety check
            return getOdometryDistanceMeters(); 
        }
        return distMeters;
    }

    private double getOdometryDistanceMeters() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        int targetTagID = (alliance == Alliance.Blue) ? 
                    VisionConstants.centerHubBlueTag : 
                    VisionConstants.centerHubRedTag;

        var hubPoseEntry = m_field_layout.getTagPose(targetTagID);
        
        if (hubPoseEntry.isEmpty()) return 0.0;

        // Get the actual Tag Position
        Translation2d tagLocation = hubPoseEntry.get().toPose2d().getTranslation();

        double centerOffsetMeters = 0.65; // Adjust this based on your specific goal depth
        double xOffset = (alliance == Alliance.Blue) ? -centerOffsetMeters : centerOffsetMeters;

        Translation2d hubCenterLocation = new Translation2d(
            tagLocation.getX() + xOffset,
            tagLocation.getY() // Keep Y the same if the tag is centered on the goal
        );

        // Calculate distance to the VIRTUAL center, not the physical tag
        return robotPose.getTranslation().getDistance(hubCenterLocation);
    }

    public double shootingDistance() {
        double distMeters = getBestDistanceMeters();
    
        if (distMeters <= 0) return 0;

        // Convert to inches for the regression formula
        double distInches = distMeters * TurretConstants.metersToInches;

        // Extra inch b/c we are aiming at the target
        double adjustedInches = distInches - 1.0; 

        // Your Regression Formula
        double targetRPS = (0.000011 * Math.pow(adjustedInches, 3)) - 
                       (0.003632 * Math.pow(adjustedInches, 2)) + 
                       (0.59999 * adjustedInches) + 32.574475;
                       
        return targetRPS;
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
            m_turret.getConfigurator().apply(turretConfig);

            lastP = P; lastD = D; lastS = S; lastPS = PS; lastDS = DS; lastVS = VS;
        }
    }

    /**
     * Main tracking logic. 
     * High-priority: Vision (TX)
     * Low-priority: Odometry (Field Position)
     */
    public void track() {
         // Identify Target
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        int targetTagID = (alliance == Alliance.Blue) ? VisionConstants.centerHubBlueTag : VisionConstants.centerHubRedTag;

        var hubPose = m_field_layout.getTagPose(targetTagID);
        if (hubPose.isEmpty()) return;

        // Get Current Robot State
        Pose2d robotPose = m_drivetrain.getState().Pose;

             // 1. Get the physical Tag Location
        Translation2d tagLocation = hubPose.get().toPose2d().getTranslation();

        boolean isCorrectTag = (LimelightHelpers.getFiducialID("limelight-four") == targetTagID);

        // 2. Calculate the Hub Center (Offsetting from the tag)
        // double centerOffset = 0.8;
        double centerOffset = 0.0;

        double xOffset = (alliance == Alliance.Blue) ? -centerOffset : centerOffset;

        Translation2d hubCenter = new Translation2d(
            tagLocation.getX() + xOffset,
            tagLocation.getY() // Usually tags are centered on the Y-axis of the goal
        );

        // 3. Use hubCenter for the Odometry-based angle calculation
        Rotation2d fieldAngle = hubCenter.minus(robotPose.getTranslation()).getAngle();
        Rotation2d robotRelativeTarget = fieldAngle.minus(robotPose.getRotation());

        double finalMotorSetpoint = 0;
        
        if (mt1 != null && mt1.tagCount > 0 && (mt1.avgTagDist < VisionConstants.acceptedAvgDistance) && (isCorrectTag)) {
            // Update odometry when sees an apriltag.
            m_visionLostCounter = 0; 

            robotPose = m_drivetrain.getState().Pose;
        }

        // --- ODOMETRY FALLBACK (Points at the hubCenter calculated above) ---
        m_visionLostCounter++;
        if (m_visionLostCounter < 5) return; 
        finalMotorSetpoint = calculateSmartWrap(robotRelativeTarget);

        double safeSetpoint = clampTurretRotations(finalMotorSetpoint);
        m_turret.setControl(m_positionRequest.withPosition(safeSetpoint));
    }

    private double clampTurretRotations(double targetRotations) {
        double leftLimitRot = (TurretConstants.leftLimit / 360.0) * TurretConstants.gearRatio;
        double rightLimitRot = (TurretConstants.rightLimit / 360.0) * TurretConstants.gearRatio;
        return MathUtil.clamp(targetRotations, rightLimitRot, leftLimitRot);
    }

    public double calculateSmartWrap(Rotation2d targetAngle) {
        // Get current motor position in DEGREES
        double currentMotorRotations = m_turret.getPosition().getValueAsDouble();
        double currentMotorDegrees = (currentMotorRotations / TurretConstants.gearRatio) * 360.0;

        // Find the closest equivalent angle to our current position (handles the "jumping" across the 180/-180 line)
        double targetDeg = targetAngle.getDegrees();
        double delta = Math.IEEEremainder(targetDeg - currentMotorDegrees, 360.0);
        double closestTarget = currentMotorDegrees + delta;

        // Check physical limits
        // If the closest target is outside the hard-stops, we have to "unwrap" it
        if (closestTarget < TurretConstants.rightLimit || closestTarget > TurretConstants.leftLimit) {
            // Try the alternative (360 degrees away)
            double altTarget = (closestTarget > currentMotorDegrees) ? closestTarget - 360 : closestTarget + 360;
        
            // If the alternative is legal, use it.
            if (altTarget >= TurretConstants.rightLimit && altTarget <= TurretConstants.leftLimit) {
                closestTarget = altTarget;
            } else {
                // Both are illegal? Clamp to the nearest soft stop.
                closestTarget = MathUtil.clamp(closestTarget, TurretConstants.rightLimit, TurretConstants.leftLimit);
            }
        }

        // Convert back to motor rotations
        return (closestTarget / 360.0) * TurretConstants.gearRatio;
    }

    public boolean isReadyToShoot() {
        boolean hasTarget = LimelightHelpers.getTV("limelight-four");
        double tx = LimelightHelpers.getTX("limelight-four");

        // Check if the motor is actually at its setpoint
        double turretMotorError = Math.abs(m_turret.getClosedLoopError().getValueAsDouble());
        double turretTolerance = (1.5 / 360.0) * TurretConstants.gearRatio;

        // We are ready if:
        // 1. We actually see a tag (not flickering)
        // 2. The crosshair error (TX) is small
        // 3. The motor has finished its move
        return (m_visionLostCounter == 0) && hasTarget && 
               (Math.abs(tx) < 2.0) && (turretMotorError < turretTolerance);
    }

    public void updateVisionOdometry() {
        double turretDegrees = getTurretDegree();
        // Limelight usually uses standard geometry where 0 is forward, CCW is positive.
        double turretRadians = Math.toRadians(turretDegrees);
        
        // Calculate the camera's position relative to the ROBOT CENTER.
        // We start at the turret center and "swing" out by the camera radius.
        double cameraX = VisionConstants.turretOffsetX - (VisionConstants.cameraRadius * Math.sin(turretRadians));
        double cameraY = VisionConstants.turretOffsetY + (VisionConstants.cameraRadius * Math.cos(turretRadians));
        
        // 1. Tell Limelight where it is physically located on the robot AT THIS MOMENT
        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-four",
            cameraX, 
            cameraY, 
            VisionConstants.altitudeMeters,
            0, 
            VisionConstants.mountedDegree, 
            turretDegrees // The yaw of the camera relative to the robot.
        );

        mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");

        publisher.set(m_drivetrain.getState().Pose);
        limeLightPublisher.set(mt1.pose);
    }

    @Override   
    public void periodic() {
        SmartDashboard.putNumber("distance", getBestDistanceMeters());
        updateVisionOdometry();
    }
}
