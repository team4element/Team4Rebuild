/*
 * This subsystem describes how the robot drives using the PhoenixTunerX constants.
 * The driver controller can indicate which speed the robot should move by: SLOW, FAST, VERY_FAST.
 * The drivetrain can use the field perspective (forward point to the opposing alliance) or the robot perspective (forward defined by swerve module from tuner constants).
 */

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public enum SPEED {
        SLOW,
        STANDARD, 
        HIGH,
        MAX 
    };

    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public SPEED m_speed = SPEED.STANDARD;

    // Blue alliance sees forward as 0 degrees (toward red alliance wall).
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    // Red alliance sees forward as 180 degrees (toward blue alliance wall). 
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    // Keep track if we've ever applied the operator perspective before or not.
    private boolean m_hasAppliedOperatorPerspective = false;

    // Variables for pathplanner configuration
    public static SwerveRequest.ApplyRobotSpeeds m_request = new SwerveRequest.ApplyRobotSpeeds();
    public static SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.1, 0.0);

    // Swerve requests to apply during SysId characterization
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final AprilTagFieldLayout m_field_layout;

    private final double m_deadband = .1;

    // This is used to get the robot's position on the field using the limelight data. It stands for MegaTag2. 
    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();
    StructPublisher<Pose2d> publisher; 
    StructPublisher<Pose2d> secondPublisher; 
    StructPublisher<Pose2d> limelightPublisher;

    public final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(m_deadband).withRotationalDeadband(m_deadband)
         .withDriveRequestType(DriveRequestType.OpenLoopVoltage
    );

    // Defines the field forward direction
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(m_deadband)
        .withRotationalDeadband(m_deadband)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage
    );

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
     @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
     @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

/**
 * Iterates through all available Limelight cameras to update the drivetrain's 
 * pose estimation using MegaTag2.
 */
public void setVisionPose() {
    updateCameraVision("limelight-four"); // Front camera
    //updateCameraVision("limelight-side"); // New side camera
    publisher.set(this.getState().Pose);

    SmartDashboard.putNumber("Odometry Distance to Hub", getOdometryDistanceMeters());
}

/**
 * Generalized MegaTag2 update logic for a single Limelight camera.
 * @param cameraName The NetworkTable name of the Limelight (e.g., "limelight-side")
 */
    private void updateCameraVision(String cameraName) {
        // Get the current robot rotation for MegaTag2's gyro-pigeon integration
        double robotYaw = this.getState().Pose.getRotation().getDegrees();
        
        // Set orientation so the Limelight can compute the field-relative pose correctly
        LimelightHelpers.SetRobotOrientation(cameraName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Retrieve the MegaTag2 Pose Estimate
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        var odometryPose = this.getState().Pose;

        limelightPublisher.set(mt2.pose);

        // 4Basic Validation
        if (mt2 == null || mt2.tagCount == 0||(mt2.pose.getX()==0&&mt2.pose.getY()==0)) {
            return; 
        }

        boolean doRejectUpdate = false;

        // Ambiguity and Distance Checks (Crucial for 1-Tag scenarios)
        if (mt2.tagCount == 1 && mt2.rawFiducials.length > 0) {
            if (mt2.rawFiducials[0].ambiguity > VisionConstants.kMaxOneTagAmbiguity) {
                doRejectUpdate = true; // Reject if the tag is blurry or skewed
            }
            if (mt2.rawFiducials[0].distToCamera > VisionConstants.kMaxOneTagDistanceMeters) {
                doRejectUpdate = true; // Reject if the tag is too far away
            }
        }

        // Teleportation Check (Prevents the "jumping" pose if vision is noisy)
        double poseDiscrepancy = mt2.pose.getTranslation().getDistance(odometryPose.getTranslation());
        if(!DriverStation.isTeleop()) {
            if (poseDiscrepancy > VisionConstants.kMaxOdometryDiscrepancyMeters) {
                doRejectUpdate = true;
            }
        }

        // Latency/Stall Check
        double dataAge = Timer.getFPGATimestamp() - mt2.timestampSeconds;
        if (dataAge > VisionConstants.kMaxDataAgeSeconds) {
            doRejectUpdate = true;
        }

        // Apply the measurement if it passed all filters
        if (!doRejectUpdate || ControllerConstants.driverController.a().getAsBoolean()) {
            // Adjust trust based on distance and tag count
            double avgDist = Math.max(mt2.avgTagDist, VisionConstants.kMinAvgTagDistFloor); 

            // Scale trust: More tags = more trust. Further distance = less trust.
            double xyStdDev = (VisionConstants.kBaselineStdDevMeters / mt2.tagCount) * avgDist * VisionConstants.kBaseTrustScale;

            // Only trust vision rotation if we see multiple tags; otherwise, rely on the Gyro
            double thetaStdDev = (mt2.tagCount >= 2) 
                ? (VisionConstants.kBaselineRotationStdDevRadians * avgDist * VisionConstants.kBaseTrustScale) 
                : VisionConstants.kUnattainableStdDev;

            this.addVisionMeasurement(
                mt2.pose,
                Utils.fpgaToCurrentTime(mt2.timestampSeconds), 
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
            
            // NetworkTableInstance.getDefault()
            //     .getStructTopic(cameraName + "/Pose", Pose2d.struct)
            //     .publish()
            //     .set(mt2.pose);
        }
    }

    /**
     * Finds the distance from the drivetrain to the face of the hub, which is determined by the alliance. 
     * @return distance of the robot from the hub.
     */
    public double getOdometryDistanceMeters() {
        Pose2d robotPose = this.getState().Pose;
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        int targetTagID = (alliance == Alliance.Blue) ? 
                    VisionConstants.centerHubBlueTag : 
                    VisionConstants.centerHubRedTag;

        var hubPoseEntry = m_field_layout.getTagPose(targetTagID);
        
        if (hubPoseEntry.isEmpty()) return 0.0;

        // Get the actual Tag Position
        Translation2d tagLocation = hubPoseEntry.get().toPose2d().getTranslation();

        // Calculate distance to the VIRTUAL center, not the physical tag
       return robotPose.getTranslation().getDistance(tagLocation);
    }

    /*
     * This creates the drivetrain within Pathplanner and allows for the coordinates to be mirrored based on alliance.
     * Returns "Pathplanner failed to work" message if function fails.
     */
    public void pathplanner(){
        try{
            RobotConfig m_config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> this.getState().Pose,// Robot pose supplier
                this::resetPose,
                ()-> {
                    return this.getState().Speeds;
                },// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                // This is is where I think the feedforward should go into!
                (speeds, feedforwards) -> this.setControl(m_request.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(7, 0.0, 0), // Translation PID constants (most likely will need tuning)
                        new PIDConstants(3, 0, 0) // Rotation PID constants (most likely would need tuning)
                ),
                m_config, // The robot configuration
                () -> {

                // Boolean supplier that controls when the path will be mirrored for the red alliance.
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements.
            ); 
        }
        catch(Exception e){
            DriverStation.reportError("PathPlanner failed to work", e.getStackTrace());
      }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices themselves. 
     * If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(AprilTagFieldLayout field_layout,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, modules);

        if(Utils.isSimulation()) {
            startSimThread();
        }

        m_field_layout = field_layout;

        pathplanner();

        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.initialIMUMode);
        
        publisher = NetworkTableInstance.getDefault().getStructTopic("botpose", Pose2d.struct).publish(); 
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices themselves. 
     * If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(AprilTagFieldLayout field_layout,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, odometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        m_field_layout = field_layout;

        pathplanner();

        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.initialIMUMode);
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct the devices themselves. 
     * If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(AprilTagFieldLayout field_layout,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        if(Utils.isSimulation()) {
            startSimThread();
        }

        m_field_layout = field_layout;

        pathplanner();

        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.initialIMUMode);

        publisher = NetworkTableInstance.getDefault().getStructTopic("botpose", Pose2d.struct).publish();
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply.
     * @return Command to run.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier){
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test.
     * @return Command to run.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test.
     * @return Command to run.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // Zeros the drivetrain's 'forward' direction.
    public Command c_seedFieldRelative(){
        return runOnce(() -> seedFieldCentric());
    }

    private void startSimThread(){
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Assigns a value to each speed mode for the drivetrain.
     * @param speed as the drive mode.
     */
    public double speedToDouble(SPEED speed){
        switch (speed) {
            case SLOW: return .20;
            case STANDARD: return .50;
            case HIGH: return .75;
            case MAX: return 1;
        }
        return 1;
    }

    /**
     * Assigns a speed to the drivetrain specified by the mode.
     * @param speed as percentage from 0 to 1.
     */
    public void setSpeed(int speed){
        if (m_speed.ordinal() + speed > SPEED.MAX.ordinal()){
            m_speed = SPEED.MAX;

        } else if (m_speed.ordinal() + speed < SPEED.SLOW.ordinal()){
            m_speed = SPEED.SLOW;

        } else {
            m_speed = SPEED.values()[m_speed.ordinal() + speed];
        }
        
        final int percent_multiplyer = 100;
        System.out.printf("Updated Speed: %s mode with: %.0f%% \r\n", m_speed.toString(), speedToDouble(m_speed) * percent_multiplyer);
    }

    /**
     * Runs a command to supply the speed mode to the drivetrain.
     * @param change +- 1 if the speed should go up or down
     * @return the command.
     */
    public Command c_updateSpeed(int change){
        return runOnce(() -> setSpeed(change));
    }

    @Override
    public void periodic(){
        setVisionPose();

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                               : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }     
    }
}

