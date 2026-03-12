/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private boolean debug = true;

    public Turret(CommandSwerveDrivetrain drivetrain){
        m_turret = new TalonFX(TurretConstants.turretID);
        m_shooterLeft = new TalonFX(TurretConstants.shooterLeftID);
        m_shooterRight = new TalonFX(TurretConstants.shooterRightID);

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

    public double getRPS(){
        return m_shooterLeft.getPosition().getValueAsDouble();
    }

    /*
     * Powers the turret motor through a position in rotations.
     */
    public void setYaw(double angle) {
        m_turret.setControl(m_positionRequest.withPosition(1));
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
        if(ControllerConstants.operatorController.povRight().getAsBoolean()){
             double limit = TurretConstants.leftLimit;

            if(getTurretDegree() <= limit){
                setTurretPercentage(0.1);

            }else if(getTurretDegree() >= limit){

                m_turret.set(0);

            }
        }else if(ControllerConstants.operatorController.povLeft().getAsBoolean()){
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
     * @return The distance in inches.
     */
    public double findDistance(){
        //
        double degreesToGoal = 0;
        if(TY < 0){
            degreesToGoal = VisionConstants.mountedDegree - (TY);
        }else if(TY >= 0){
            degreesToGoal = 12.5 + (TY);
        }
        
        double radiansToGoal = degreesToGoal *(3.141592/180);

        // The formula below can be used to find the degree the limelight is rotated backward from vertical.
        //double a1 = ((Math.atan(VisionConstants.hubApriltagHeight - VisionConstants.altitude)/36)*(180/Math.PI))-(TY);

        double distance = (VisionConstants.hubApriltagHeight - VisionConstants.altitude)/Math.tan(radiansToGoal);

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
        m_shooterLeft.setControl(m_voltage.withVelocity(shootingDistance()).withSlot(0));
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

    public void periodic(){
        hasTarget = LimelightHelpers.getTV("limelight-four");
        TY = LimelightHelpers.getTY("limelight-four");
        TX = LimelightHelpers.getTX("limelight-four");

        if(debug){
            double turretP = SmartDashboard.getNumber("Turret kP", TurretConstants.KPTurret); 
            double turretD = SmartDashboard.getNumber("Turret kD", TurretConstants.KDTurret);
            double turretS = SmartDashboard.getNumber("Turret kS", TurretConstants.KSTurret);
            double shooterP = SmartDashboard.getNumber("Shooter kP", TurretConstants.KPShooter); 
            double shooterD = SmartDashboard.getNumber("Shooter kD", TurretConstants.KDShooter);
            double shooterV = SmartDashboard.getNumber("Shooter kV", TurretConstants.KVShooter);
            SmartDashboard.putNumber("turret pos", m_turret.getPosition().getValueAsDouble());
            //SmartDashboard.putNumber("shooter speed", m_shooterLeft.getVelocity().getValueAsDouble());

            updateValues(turretP, turretD, turretS, shooterP, shooterD, shooterV);
        }
       //SmartDashboard.putNumber("TY", TY);
       //SmartDashboard.putNumber("turret degree", findDistance());
    }
}
