/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase{
    // Declares motors and sensors.
    private TalonFX m_turret, m_shooterLeft, m_shooterRight;

    // Used to control speed of motors.
    private DutyCycleOut m_dutyCycleTurret;
    private CurrentLimitsConfigs m_limitConfigTurret = new CurrentLimitsConfigs();
    private CurrentLimitsConfigs m_limitConfigShooter = new CurrentLimitsConfigs();
    private PositionVoltage m_positionRequest;
    @SuppressWarnings("unused")
    private SimpleMotorFeedforward m_feedForward;
    private VelocityVoltage m_voltage;

    NetworkTable table;

    TalonFXConfiguration shooterConfig;
    TalonFXConfiguration turretConfig;

    ClosedLoopGeneralConfigs generalConfigs;

    CommandSwerveDrivetrain m_drivetrain;

    private int m_visionLostFrames = 0;

    // Variables used to update values
    double lastP;
    double lastD;
    double lastS;
    double lastPS;
    double lastDS;
    double lastVS;

    // Declares x and y offsets from limelight
    private double TX, TY;
    // Starts the robot's distance from the hub at 0 (since we will be starting flush to the hub).
    private double distance = 0;
    // Determines weather or not limelight sees apriltag
    private boolean hasTarget;

    // This should be true only when you want to tune constants through the shuffleboard.
    @SuppressWarnings("unused")
    private boolean debug = false;

    // Used to get the turret's position on field relative to limelight.
    private AprilTagFieldLayout m_field_layout;

    public Turret(AprilTagFieldLayout field_layout, CommandSwerveDrivetrain drivetrain){
        m_turret = new TalonFX(TurretConstants.turretID);
        m_shooterLeft = new TalonFX(TurretConstants.shooterLeftID);
        m_shooterRight = new TalonFX(TurretConstants.shooterRightID);
        m_field_layout = field_layout;

        m_drivetrain = drivetrain;

        // The turret and shooter motor will start with half speed
        m_dutyCycleTurret = new DutyCycleOut(TurretConstants.dutyCycleTurret);
        m_feedForward = new SimpleMotorFeedforward(TurretConstants.KSTurret, TurretConstants.KVTurret);

        m_positionRequest = new PositionVoltage(0).withSlot(0);
        m_voltage = new VelocityVoltage(0).withSlot(0);

        turretConfig = new TalonFXConfiguration();
        shooterConfig = new TalonFXConfiguration();

        // Assigns PID values to the turret for precise speed
        turretConfig.Slot0.kP = TurretConstants.KPTurret; // Controls position error
        turretConfig.Slot0.kI = 0; // Controls integral error using kP and kD (don't change)
        turretConfig.Slot0.kD = TurretConstants.KDTurret; // Controls derivative error
        turretConfig.Slot0.kS = TurretConstants.KSTurret;

        MotionMagicConfigs mmConfigs = turretConfig.MotionMagic;
        // 1. Cruise Velocity: How fast should the turret spin at top speed?
        // Start slow (e.g., 40 RPS) and increase until it's fast enough to track.
        mmConfigs.MotionMagicCruiseVelocity = TurretConstants.turretMaxVelocity;

        // 2. Acceleration: How fast should it reach top speed?
        // Usually, set this to 2x or 4x your Cruise Velocity for a snappy feel.
        mmConfigs.MotionMagicAcceleration = TurretConstants.turretMaxAcceleration;

        // 3. Jerk: How smooth should the start/stop be?
        // 0 is a trapezoidal profile. 50-200 adds an "S-Curve" to prevent belt skipping.
        mmConfigs.MotionMagicJerk = TurretConstants.turretMaxJerk;

        // Assigns PID values to the shooter for precise speed
        shooterConfig.Slot0.kP = TurretConstants.KPShooter; // Controls position error
        shooterConfig.Slot0.kI = TurretConstants.KIShooter; // Controls integral error using kP and kD (don't change)
        shooterConfig.Slot0.kD = TurretConstants.KDShooter; // Controls derivative error
        shooterConfig.Slot0.kV = TurretConstants.KVShooter;

        lastP = TurretConstants.KPTurret;
        lastD = TurretConstants.KDTurret;
        lastS = TurretConstants.KSTurret;
        lastPS = TurretConstants.KPShooter;
        lastDS = TurretConstants.KDShooter;
        lastVS = TurretConstants.KVShooter;

        m_turret.getConfigurator().apply(turretConfig);

        shooterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        m_shooterLeft.getConfigurator().apply(shooterConfig);

        m_shooterRight.setControl(new Follower(TurretConstants.shooterLeftID, MotorAlignmentValue.Opposed));

        // Motor current limits to test for later.
        m_limitConfigTurret = new CurrentLimitsConfigs();
        TalonFXConfigurator configuratorTurret = m_turret.getConfigurator();
        m_limitConfigTurret.StatorCurrentLimit = TurretConstants.turretStatorLimit;
        m_limitConfigTurret.StatorCurrentLimitEnable = true;
        configuratorTurret.apply(m_limitConfigTurret);

        m_limitConfigShooter = new CurrentLimitsConfigs();
        TalonFXConfigurator configuratorShooter = m_turret.getConfigurator();
        m_limitConfigShooter.StatorCurrentLimit = TurretConstants.shooterStatorLimit;
        m_limitConfigShooter.StatorCurrentLimitEnable = true;
        configuratorShooter.apply(m_limitConfigShooter);

        // Puts a filter on what tags the limelight will recognize.
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", VisionConstants.validIDS);

        // Puts the constants onto the shuffleboard which will update periodically.
        SmartDashboard.putNumber("Turret kP", TurretConstants.KPTurret);
        SmartDashboard.putNumber("Turret kD", TurretConstants.KDTurret);
        SmartDashboard.putNumber("Turret kS", TurretConstants.KSTurret);
        SmartDashboard.putNumber("Shooter kP", TurretConstants.KPShooter);
        SmartDashboard.putNumber("Shooter kD", TurretConstants.KDShooter);
        SmartDashboard.putNumber("Shooter kV", TurretConstants.KVShooter);
        LimelightHelpers.SetIMUMode("limelight-four",0);
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
        m_turret.setControl(m_voltage.withVelocity(RPS).withSlot(0));
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
        return m_shooterLeft.getPosition().getValueAsDouble();
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
        m_turret.setNeutralMode(NeutralModeValue.Brake);
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

            if(distance>=TurretConstants.distanceUpperLimit || distance<=TurretConstants.distanceLowerLimit){
                distance = 0;

            }
        }  
        return distance;
    }

    /**
     * Calculates the speed that the shooter motor should spin in order to reach a target using a formula generated through regression.
     * If the distance found by the previous function returns 0, the speed of the motor will be 0.
     * @return rotations per second.
     */
    public double shootingDistance(){
        distance = findDistance()*TurretConstants.metersToInches;
        double RPS;

        if(distance == 0){
           RPS = 0;

        }else{
            // This is the formula found by regression in MatLab using RPS for the motor and inches it reaches.
            RPS = (0.000011*Math.pow(distance,3))-(0.003632*Math.pow(distance,2))+(0.59999*distance)+32.574475;

        }

        return RPS;
    }

    /**
     * Assigns a speed to run the shooter motor using PID.
     * @param RPS from -1 to 1.
     */
    public void startShooter(double distance){
        m_shooterLeft.setControl(m_voltage.withVelocity(distance).withSlot(0));
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

            lastP = P;
            lastD = D;
            lastS = S;
            lastPS = PS;
            lastDS = DS;
            lastVS = VS;
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

    /**
     * Checks if the turret is properly lined up to shoot successfully.
     * @return shooting status.
     */
    public boolean isReadyToShoot() {
        double error = Math.abs(m_turret.getClosedLoopError().getValueAsDouble());
        double shooterError = Math.abs(m_shooterLeft.getClosedLoopError().getValueAsDouble());

        // Within ~1 degree of target and ~2 RPS of target shooter speed
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
        updateVisionOdometry();
    }


        // public void periodic(){
    //     //hasTarget = LimelightHelpers.getTV("limelight-four");

    //     if(debug){
    //         double turretP = SmartDashboard.getNumber("Turret kP", TurretConstants.KPTurret);
    //         double turretD = SmartDashboard.getNumber("Turret kD", TurretConstants.KDTurret);
    //         double turretS = SmartDashboard.getNumber("Turret kS", TurretConstants.KSTurret);
    //         double shooterP = SmartDashboard.getNumber("Shooter kP", TurretConstants.KPShooter);
    //         double shooterD = SmartDashboard.getNumber("Shooter kD", TurretConstants.KDShooter);
    //         double shooterV = SmartDashboard.getNumber("Shooter kV", TurretConstants.KVShooter);
    //         double RPS = SmartDashboard.getNumber("Shooter RPS", TurretConstants.shooterSpeed);
    //         SmartDashboard.putNumber("turret degree", getTurretDegree());
    //         SmartDashboard.putNumber("shooter speed", m_shooterLeft.getVelocity().getValueAsDouble());

    //         updateValues(turretP, turretD, turretS, shooterP, shooterD, shooterV, RPS);
    //     }
    //    SmartDashboard.putNumber("distance", findDistance());
    //    SmartDashboard.putNumber("shooter speed", m_shooterLeft.getVelocity().getValueAsDouble());
    // }
}
