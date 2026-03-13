/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    // Declares motors and sensors
    private TalonFX m_turret, m_shooterLeft, m_shooterRight;

    // Used to control speed of motors
    private DutyCycleOut m_dutyCycleTurret;
    //private CurrentLimitsConfigs m_limitConfigTurret = new CurrentLimitsConfigs();
    //private CurrentLimitsConfigs m_limitConfigShooter = new CurrentLimitsConfigs();
    private PositionVoltage m_positionRequest;
    @SuppressWarnings("unused")
    private SimpleMotorFeedforward m_feedForward;
    private VelocityVoltage m_voltage;

    NetworkTable table;

    TalonFXConfiguration shooterConfig;
    TalonFXConfiguration turretConfig;

    ClosedLoopGeneralConfigs generalConfigs;

    CommandSwerveDrivetrain m_drivetrain;

    // Variables used to update values
    double lastRPS;
    double lastP;
    double lastD;
    double lastS;
    double lastPS;
    double lastDS;
    double lastVS;

    // Declares x and y offsets from limelight
    private double TX, TY;
    // Determines weather or not limelight sees apriltag
    private boolean hasTarget; 

    // This should be true only when you want to tune constants through the shuffleboard.
    private boolean debug = false;

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

        lastRPS = TurretConstants.shooterSpeed;
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

        // TalonFXConfigurator configuratorTurret = m_turret.getConfigurator();
        // m_limitConfigTurret.StatorCurrentLimit = TurretConstants.turretStatorLimit;
        // m_limitConfigTurret.StatorCurrentLimitEnable = true;
        // m_limitConfigShooter.SupplyCurrentLimit = TurretConstants.turretSupplyLimit;
        // m_limitConfigShooter.SupplyCurrentLimitEnable = true;
        // configuratorTurret.apply(m_limitConfigTurret);

        // TalonFXConfigurator configuratorShooter = m_turret.getConfigurator();
        // m_limitConfigShooter.StatorCurrentLimit = TurretConstants.shooterStatorLimit;
        // m_limitConfigShooter.StatorCurrentLimitEnable = true;
        // m_limitConfigShooter.SupplyCurrentLimit = TurretConstants.shooterSupplyLimit;
        // m_limitConfigShooter.SupplyCurrentLimitEnable = true;
        // configuratorShooter.apply(m_limitConfigShooter);

        // Puts a filter on what tags the limelight will recognize. 
        //LimelightHelpers.SetFiducialIDFiltersOverride("limelight-four", VisionConstants.validIDS);

        // Puts the constants onto the shuffleboard which will update periodically.
        SmartDashboard.putNumber("Turret kP", TurretConstants.KPTurret);
        SmartDashboard.putNumber("Turret kD", TurretConstants.KDTurret);
        SmartDashboard.putNumber("Turret kS", TurretConstants.KSTurret);
        SmartDashboard.putNumber("Shooter kP", TurretConstants.KPShooter);
        SmartDashboard.putNumber("Shooter kD", TurretConstants.KDShooter);
        SmartDashboard.putNumber("Shooter kV", TurretConstants.KVShooter);
        SmartDashboard.putNumber("Shooter RPS", TurretConstants.shooterSpeed);
        LimelightHelpers.SetIMUMode("limelight-four",4);
    }

    /*
     * Sets the turret's starting position.
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

    /*
     * Powers the turret motor through a position in rotations.
     */
    public void setYaw(double angle) {
        m_turret.setControl(m_positionRequest.withPosition(angle));
    }

    /*
     * Stops both the turret and shooter movement.  
     */
    public void stopMotors(){
        m_turret.setControl(m_dutyCycleTurret.withOutput(0));
        m_shooterLeft.setControl(m_dutyCycleTurret.withOutput(0));
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
        return m_turret.getPosition().getValue().in(Degrees)/TurretConstants.gearRatio; 
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
        if(ControllerConstants.operatorController.b().getAsBoolean()){
             double limit = TurretConstants.leftLimit;

            if(getTurretDegree() <= limit){
                setTurretPercentage(0.1);

            }else if(getTurretDegree() >= limit){

                m_turret.set(0);

            }
        }else if(ControllerConstants.operatorController.x().getAsBoolean()){
             double limit = TurretConstants.rightLimit;

            if(getTurretDegree() <= limit){
                m_turret.set(0);

            }else if(getTurretDegree() >= limit){
                setTurretPercentage(-0.1);

            }
        }else{
            m_turret.set(0);
        }
    }

    /**
     * Finds the distance between the apriltag and the limelight (center of lens).
     * If the distance is more than the shooter's distance limit, the distance will return 0.
     * @link https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance  -> Explains the calculations to find the distance.
     * @return The distance as a degree in radians.
     */
    public double findDistance(){
        double degreesToGoal = VisionConstants.mountedDegree + (TY);
        double radiansToGoal = Math.toRadians(degreesToGoal);

        // The formula below can be used to find the degree the limelight is rotated backward from vertical.
        //double a1 = ((Math.atan(VisionConstants.hubApriltagHeight - VisionConstants.altitude)/100)*(180/Math.PI))-(TY);

        double distance = (VisionConstants.hubApriltagHeightMeters - VisionConstants.altitudeMeters)/Math.tan(radiansToGoal);

        if(distance>=TurretConstants.distanceUpperLimit || distance<=TurretConstants.distanceLowerLimit){
            distance = 0;

        }

       return distance;
    }

    /**
     * Calculates the speed that the shooter motor should spin in order to reach a target using a formula generated through regression.
     * If the distance found by the previous function returns 0, the speed of the motor will be 0.
     * @return rotations per second.
     */
    public double shootingDistance(){
        double distance = findDistance();
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
    public void startShooter(){
        m_shooterLeft.setControl(m_voltage.withVelocity(TurretConstants.shooterSpeed).withSlot(0));
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
    public void updateValues(double P, double D, double S, double PS, double DS, double VS, double RPS){
        if(P != lastP || D != lastD || S != lastS || PS != lastPS || DS != lastDS || VS != lastVS || RPS != lastRPS){
            turretConfig.Slot0.kP = P;
            turretConfig.Slot0.kD = D;
            turretConfig.Slot0.kS = S;
            shooterConfig.Slot0.kP = PS;
            shooterConfig.Slot0.kD = DS;
            shooterConfig.Slot0.kV = VS;
            m_turret.getConfigurator().apply(turretConfig);
            m_shooterLeft.getConfigurator().apply(shooterConfig);

            TurretConstants.shooterSpeed = RPS;

            lastP = P;
            lastD = D;
            lastS = S;
            lastPS = PS;
            lastDS = DS;
            lastVS = VS;
            lastRPS = RPS;
        }
    }

    public void trackAndShoot() {
        // 1. Get current robot pose and field-relative velocity
        Pose2d robotPose = m_drivetrain.getState().Pose;
        
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_drivetrain.getState().Speeds, 
            robotPose.getRotation()
        );

        // 2. Calculate Distance to Hub
        var hub = m_field_layout.getTagPose(26).get().toPose2d();
        double distance = robotPose.getTranslation().getDistance(hub.getTranslation());
    
        // 3. LEAD COMPENSATION: Calculate where to aim while moving
        // shotVelocity: horizontal speed of the ball (m/s)
        double shotVelocity = 15.0; 
        double timeToGoal = distance / shotVelocity;

        // Virtual target moves opposite to our robot's velocity
        double virtualX = hub.getX() - (chassisSpeeds.vxMetersPerSecond * timeToGoal);
        double virtualY = hub.getY() - (chassisSpeeds.vyMetersPerSecond * timeToGoal);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        System.out.println("hubX: " + hub.getX() + "hubY: " + hub.getY() + "X: " + robotPose.getX() + " Y: " + robotPose.getY() + " Xv: " + virtualX + " Yv: " + virtualY);

        // 4. Calculate Angles
        Rotation2d fieldAngle = virtualTarget.minus(robotPose.getTranslation()).getAngle();
        Rotation2d robotRelativeTarget = fieldAngle.minus(robotPose.getRotation());

        // 5. Apply Smart Wrap & Gear Ratio
        double motorRotations = calculateSmartWrap(robotRelativeTarget);

        // 6. Set Motor with Motion Magic (for smooth tracking)
        m_turret.setControl(m_positionRequest.withPosition(motorRotations));
    
        // 7. Auto-Spin Shooter based on distance
        double targetRPS = shootingDistance(); 
        m_shooterLeft.setControl(m_voltage.withVelocity(targetRPS));
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

    public double calculateSmartWrap(Rotation2d targetAngle) {
        double targetDeg = targetAngle.getDegrees(); // The "ideal" robot-relative angle

        // 1. Standardize target to -180 to 180
        while (targetDeg > 180) targetDeg -= 360;
        while (targetDeg < -180) targetDeg += 360;

        // 2. Check if current target is within limits
        if (targetDeg >= TurretConstants.rightLimit && targetDeg <= TurretConstants.leftLimit) {
            return (targetDeg / 360.0) * TurretConstants.gearRatio;
        }

        // 3. If not, check the "other side" (add/subtract 360)
        double alternativeSide;
        if (targetDeg > 0) {
            alternativeSide = targetDeg - 360;
        } else {
            alternativeSide = targetDeg + 360;
        }

        // 4. If the alternative is within limits, use it. 
        // Otherwise, clamp to the closest hard stop.
        if (alternativeSide >= TurretConstants.rightLimit && alternativeSide <= TurretConstants.leftLimit) {
        return (alternativeSide / 360.0) * TurretConstants.gearRatio;
        } else {
            return (MathUtil.clamp(targetDeg, TurretConstants.rightLimit, TurretConstants.leftLimit) / 360.0) * TurretConstants.gearRatio;
        }
    }

    public boolean isReadyToShoot() {
        double error = Math.abs(m_turret.getClosedLoopError().getValueAsDouble());
        double shooterError = Math.abs(m_shooterLeft.getClosedLoopError().getValueAsDouble());
    
        // Within ~1 degree of target and ~2 RPS of target shooter speed
        return (error < (1.0 / 360.0) * TurretConstants.gearRatio) && (shooterError < 2.0);
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

    public void updateVisionOdometry() {
        // 1. Get the current turret angle in degrees
        double turretDegrees = getTurretDegree(); 

        // 2. Dynamically update the camera's position relative to the ROBOT center
        // We pass the turret's rotation as the YAW (the last parameter)
        LimelightHelpers.setCameraPose_RobotSpace("limelight-four", 
            VisionConstants.forwardOffsetMeters, 
            VisionConstants.sideOffsetMeters, 
            VisionConstants.altitudeMeters, 
            VisionConstants.mountedDegree, 
            0, // Roll
            turretDegrees // Yaw (this is the key!)
        );

        // 3. Get the Pose Estimate using MegaTag2
        // It needs the robot's current gyro yaw to stabilize the vision data
        double robotYaw = m_drivetrain.getState().Pose.getRotation().getDegrees();

        // Push the Pigeon 2.0 yaw to the Limelight so MegaTag2 can use it
        LimelightHelpers.SetRobotOrientation("limelight-four", robotYaw, 0, 0, 0, 0, 0);

        // Now the single-parameter method will work correctly
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four");

        // 4. Feed the result into your Phoenix 6 Swerve Drivetrain
        if (mt2 != null && mt2.tagCount > 0) {
            // Calculate how far our current Odometry is from what the Vision sees
            double distanceError = m_drivetrain.getState().Pose.getTranslation().getDistance(mt2.pose.getTranslation());

            // If we are more than 2 meters away from where we should be, "Seed" (Teleport) the robot.
            // This fixes your X = -3.8 issue instantly the moment it sees a tag.
            if (distanceError > 2.0) {
                m_drivetrain.c_seedFieldRelativeWithVision();
            } else {
                // If we are already close, just blend the vision in normally
                m_drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    @Override
    public void periodic() {
        updateVisionOdometry();
    }
}
