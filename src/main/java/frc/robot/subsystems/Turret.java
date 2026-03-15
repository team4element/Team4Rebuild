/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase {
    // Hardware
    private final TalonFX m_turret;
    private final TalonFX m_shooterLeft;
    private final TalonFX m_shooterRight;

    // Control Requests
    private final DutyCycleOut m_dutyCycleTurret;
    private final PositionVoltage m_positionRequest;
    private final VelocityVoltage m_velocityRequest; 

    // Configurations
    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    // Dependencies
    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagFieldLayout m_field_layout;

    // Logic State
    private int m_visionLostFrames = 0;
    private double lastP, lastD, lastS, lastPS, lastDS, lastVS;
    private double TX, TY;
    private double distance = 0;
    private boolean hasTarget;
    @SuppressWarnings("unused")
    private boolean debug = false;

    public Turret(AprilTagFieldLayout field_layout, CommandSwerveDrivetrain drivetrain) {
        m_turret = new TalonFX(TurretConstants.turretID);
        m_shooterLeft = new TalonFX(TurretConstants.shooterLeftID);
        m_shooterRight = new TalonFX(TurretConstants.shooterRightID);
        m_field_layout = field_layout;
        m_drivetrain = drivetrain;

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
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_turret.getConfigurator().apply(turretConfig);

        // --- Shooter Config ---
        shooterConfig.Slot0.kP = TurretConstants.KPShooter;
        shooterConfig.Slot0.kI = TurretConstants.KIShooter;
        shooterConfig.Slot0.kD = TurretConstants.KDShooter;
        shooterConfig.Slot0.kV = TurretConstants.KVShooter;
    
        shooterConfig.Feedback.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RotorSensor;
    
        shooterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
        shooterConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.shooterStatorLimit;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_shooterLeft.getConfigurator().apply(shooterConfig);
        m_shooterRight.getConfigurator().apply(shooterConfig);

        m_shooterRight.setControl(new Follower(m_shooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        lastP = TurretConstants.KPTurret;
        lastD = TurretConstants.KDTurret;
        lastS = TurretConstants.KSTurret;
        lastPS = TurretConstants.KPShooter;
        lastDS = TurretConstants.KDShooter;
        lastVS = TurretConstants.KVShooter;

        SmartDashboard.putNumber("Turret kP", lastP);
        SmartDashboard.putNumber("Shooter kV", lastVS);
    
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
     * Gets the rotation per second of shooter leader motor.
     * @return RPS.
     */
    public double getRPS(){
        return m_shooterLeft.getVelocity().getValueAsDouble();
    }

    /**
     * Powers the turret motor through a position in rotations.
     * @param angle from 0 to 360.
     */
    public void setYaw(double angle) {
        if(angle <= 210 || angle >= 49){
          m_turret.setControl(m_positionRequest.withPosition(angle));
        }
    }

    /*
     * Stops both the turret and shooter movement.
     */
    public void stopMotors(){
        m_turret.setControl(m_dutyCycleTurret.withOutput(0));
        m_shooterLeft.setControl(m_dutyCycleTurret.withOutput(0));
        m_shooterRight.setControl(m_dutyCycleTurret.withOutput(0));
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
        return m_turret.getPosition().getValueAsDouble()/TurretConstants.gearRatio;
    }

    /**
     * Allows the operator to control the direction of the turret.
     * Stops the motor once the turret reaches the left or right limits.
     */
    public void rotateManual(){
        if(ControllerConstants.operatorController.povLeft().getAsBoolean()){
             double limit = TurretConstants.leftLimit;

            if(getTurretDegree() <= limit){
                setTurretPercentage(-0.1);

            }else if(getTurretDegree() >= limit){

                m_turret.set(0);

            }
        }else if(ControllerConstants.operatorController.povRight().getAsBoolean()){
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
     * Finds the distance between the apriltag and the limelight (center of lens).
     * If the distance is more than the shooter's distance limit, the distance will return 0.
     * @link https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance  -> Explains the calculations to find the distance.
     * @return The distance in meters.
     */
    public double findDistance(){
        if(hasTarget){
            double degreesToGoal = VisionConstants.mountedDegree + TY;          
            double radiansToGoal = degreesToGoal * (Math.PI/180);

            // The formula below can be used to find the degree the limelight is rotated backward from vertical.
            //double a1 = ((Math.atan(VisionConstants.hubApriltagHeight - VisionConstants.altitude)/36)*(180/Math.PI))-(TY);

            distance = (VisionConstants.hubApriltagHeightMeters - VisionConstants.altitudeMeters)/Math.tan(radiansToGoal);

            // if(distance>=TurretConstants.distanceUpperLimit || distance<=TurretConstants.distanceLowerLimit){
            //     distance = 0;

            // }
        }  
        return distance;
    }


    public double shootingDistance() {
        double distInches = findDistance() * TurretConstants.metersToInches;
        if (distInches == 0) return 0;
    
        // This result should be in RPS (Rotations Per Second)
        double targetRPS = (0.000011 * Math.pow(distInches, 3)) - 
                       (0.003632 * Math.pow(distInches, 2)) + 
                       (0.59999 * distInches) + 32.574475;
                       
        return targetRPS;
    }

    public void startShooter(double RPS) {
        System.out.println(RPS +  " | " + m_shooterLeft.getVelocity().toString());
        m_shooterLeft.setControl(m_velocityRequest.withVelocity(RPS).withSlot(0));
    }


    /**
     * Gets the degree the robot needs to turn to get to center of apriltag if the apritag is detected.
     * Results in an error if the apriltag isn't there.
     * @return The degree in radians.
     */
    public double findAngleToTarget(){
        if(hasTarget == true){
            double distanceToApriltag = findDistance();
            double xAngleInRadians = Math.toRadians(TX);

            double xTarget = distanceToApriltag*Math.sin(xAngleInRadians);
            double yTarget = distanceToApriltag*Math.cos(xAngleInRadians);

            double angle = Math.atan2(xTarget, yTarget);
            return angle;
        }else{
            int apriltagNotFound = 0;
            System.out.println("ERROR: APRILTAG NOT FOUND");

            return apriltagNotFound;
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
            shooterConfig.Slot0.kP = PS;
            shooterConfig.Slot0.kD = DS;
            shooterConfig.Slot0.kV = VS;
            m_turret.getConfigurator().apply(turretConfig);
            m_shooterLeft.getConfigurator().apply(shooterConfig);
            m_shooterRight.getConfigurator().apply(shooterConfig);

            lastP = P; lastD = D; lastS = S; lastPS = PS; lastDS = DS; lastVS = VS;
        }
    }

    public void trackAndShoot() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
        var hubPose = m_field_layout.getTagPose(26);
        if (hubPose.isEmpty()) return;

        Translation2d hubLocation = hubPose.get().toPose2d().getTranslation();
    
        // Calculate the 'Ideal' Robot-Relative Angle
        // This is where the turret SHOULD be based on the map.
        Rotation2d fieldAngle = hubLocation.minus(robotPose.getTranslation()).getAngle();

        Rotation2d robotRelativeTarget = fieldAngle.minus(robotPose.getRotation());

        double finalMotorSetpoint;

        // Priority 1: Live Vision
        if (LimelightHelpers.getTV("limelight-four")) {
            m_visionLostFrames = 0;
        
            double currentMotorRotations = m_turret.getPosition().getValueAsDouble();
            double tx = LimelightHelpers.getTX("limelight-four");
        
            // Convert TX to motor rotations
            double motorError = (tx / 360.0) * TurretConstants.gearRatio;

            // Directly adjust based on what the camera sees
            finalMotorSetpoint = currentMotorRotations - motorError;

        } else {
            // Priority 2: Odometry (The Fallback) Estimate the position based on the Supplied Welded Map
            m_visionLostFrames++;
        
            if (m_visionLostFrames < 5) {
                // Stay put for a split second to avoid "flicker jerks"
                finalMotorSetpoint = m_turret.getPosition().getValueAsDouble();
            } else {
                // Use the Smart Wrap logic to find the best way home
                finalMotorSetpoint = calculateSmartWrap(robotRelativeTarget);
            }
        }

        // Prevent turret from exceeding limits
        double safeSetpoint = clampTurretRotations(finalMotorSetpoint);

        // Tell Turret to Move
        m_turret.setControl(m_positionRequest.withPosition(safeSetpoint));    
    }

    private double calculateConstrainedRotation(Rotation2d target) {
        double val = target.getRotations(); // Phoenix 6 likes rotations

        // Simple wrapping logic: If target is 0.6 rotations (216 deg),
        // and limit is 0.5 (180 deg), check if -0.4 rotations (-144 deg) is valid.
        if (val > (TurretConstants.leftLimit / 360.0)) val -= 1.0;
        if (val < (TurretConstants.rightLimit / 360.0)) val += 1.0;

        // Final safety clamp
        return MathUtil.clamp(val, TurretConstants.rightLimit / 360.0, TurretConstants.leftLimit / 360.0);
    }

    private double clampTurretRotations(double targetRotations) {
        double leftLimitRot = (TurretConstants.leftLimit / 360.0) * TurretConstants.gearRatio;
        double rightLimitRot = (TurretConstants.rightLimit / 360.0) * TurretConstants.gearRatio;
        return MathUtil.clamp(targetRotations, rightLimitRot, leftLimitRot);
    }

    /**
     * This makes sure that the turret won't go past the physical limits by snapping to the other side. 
     * @param targetAngle
     * @return new turret angle.
     */
    public double calculateSmartWrap(Rotation2d targetAngle) {
    // Standardize the target to be between -180 and 180
    double targetDeg = targetAngle.getDegrees();
    while (targetDeg > 180) targetDeg -= 360;
    while (targetDeg < -180) targetDeg += 360;

    // Check if it's within your physical constants
    // If TurretConstants.rightLimit is -150 and target is -170, it's out of bounds
    if (targetDeg < TurretConstants.rightLimit || targetDeg > TurretConstants.leftLimit) {
        // It's outside the "legal" zone. Try the other way around.
        double altTarget = (targetDeg > 0) ? targetDeg - 360 : targetDeg + 360;
        
        // If the alternative is legal, use it. Otherwise, clamp to the nearest edge.
        if (altTarget >= TurretConstants.rightLimit && altTarget <= TurretConstants.leftLimit) {
            targetDeg = altTarget;
        } else {
            targetDeg = MathUtil.clamp(targetDeg, TurretConstants.rightLimit, TurretConstants.leftLimit);
        }
    }
        return (targetDeg / 360.0) * TurretConstants.gearRatio;
    }

    public boolean isReadyToShoot() {
        double error = Math.abs(m_turret.getClosedLoopError().getValueAsDouble());
        double shooterError = Math.abs(m_shooterLeft.getClosedLoopError().getValueAsDouble());
        return (error < (1.0 / 360.0) * TurretConstants.gearRatio) && (shooterError < 2.0);
    }

    public void updateVisionOdometry() {
        // Get the angle of the turret in degrees where 0 is facing "forward"
        double turretDegrees = getTurretDegree();
        // Get the rotation of the drivetrain where 0 is forwars
        double robotYaw = m_drivetrain.getState().Pose.getRotation().getDegrees();

        //Update the position of the camera since it is mounted to the turret (which spins)
        double pivotX = 0.16;
        double pivotY = 0.16; 

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight-four",
            pivotX, pivotY, VisionConstants.altitudeMeters,
            VisionConstants.mountedDegree, 0, turretDegrees
        );

        // Tells the Limelight which way the drive train is facing.
        LimelightHelpers.SetRobotOrientation(
            "limelight-four",
            robotYaw, // Degrees
            0, 0, 0, 0, 0
        );

        // Use the MegaTag 2 Pose
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");

        if (mt2 != null && mt2.tagCount > 0) {
            m_drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
        }
    }

    @Override   
    public void periodic() {
        // 1. Refresh vision state so findDistance() and trackAndShoot() use fresh data
        hasTarget = LimelightHelpers.getTV("limelight-four");
        if (hasTarget) {
            TX = LimelightHelpers.getTX("limelight-four");
            TY = LimelightHelpers.getTY("limelight-four");
        }

        SmartDashboard.putNumber("distance", findDistance());
        updateVisionOdometry();
    }
}