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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    public enum SPEED {
        SLOW, // Runs the wheels at half speed
        FAST, // Runs the wheels at 75 percent speed
        VERY_FAST // Runs the wheels at full speed
    };

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private int isBooleanTrue;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean robotFieldCentric;
    public SPEED m_speed = SPEED.FAST;

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

    // This is used to get the robot's position on the field using the limelight data. It stands for MegaTag2. 
    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();
    StructPublisher<Pose2d> publisher; 
    StructPublisher<Pose2d> limelightPublisher;
    StructPublisher<Pose2d> publisher_2;

    public final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(.6).withRotationalDeadband(.6)
         .withDriveRequestType(DriveRequestType.OpenLoopVoltage
    );

    // Defines the field forward direction
    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(.2)
        .withRotationalDeadband(.2)
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

    public void setVisionPose() {
        double robotYaw = this.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);
        var odometryPose = this.getState().Pose;

        publisher.set(odometryPose);

        // Tag Count Check
        if (mt2 == null || mt2.tagCount == 0) {
            return; 
        }

        limelightPublisher.set(mt2.pose);
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

        // Teleportation Check 
        double poseDiscrepancy = mt2.pose.getTranslation().getDistance(odometryPose.getTranslation());
        if (poseDiscrepancy > VisionConstants.kMaxOdometryDiscrepancyMeters) {
            doRejectUpdate = true;
        }

        // Stall/Lag Check (Data must be fresh)
        double dataAge = Timer.getFPGATimestamp() - mt2.timestampSeconds;
        if (dataAge > VisionConstants.kMaxDataAgeSeconds) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            double avgDist = Math.max(mt2.avgTagDist, VisionConstants.kMinAvgTagDistFloor); 

            // More trust if we see more tags
            double xyStdDev = (VisionConstants.kBaselineStdDevMeters / mt2.tagCount) * avgDist * VisionConstants.kBaseTrustScale;

            double thetaStdDev = (mt2.tagCount >= 2) 
                ? (VisionConstants.kBaselineRotationStdDevRadians * avgDist * VisionConstants.kBaseTrustScale) 
                : VisionConstants.kUnattainableStdDev;

            this.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds, 
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
        }
        
        publisher_2.set(this.getState().Pose);
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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, modules);

        if(Utils.isSimulation()) {
            startSimThread();
        }

        pathplanner();

        robotFieldCentric = false;
        isBooleanTrue = -1;
        
        publisher = NetworkTableInstance.getDefault().getStructTopic("botpose", Pose2d.struct).publish(); 
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
        publisher_2 = NetworkTableInstance.getDefault().getStructTopic("correctedPose", Pose2d.struct).publish();
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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, odometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        pathplanner();

        LimelightHelpers.SetIMUMode("limelight-four", 4);

        robotFieldCentric = false;
        isBooleanTrue = -1;

        publisher = NetworkTableInstance.getDefault().getStructTopic("botpose", Pose2d.struct).publish();
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
        publisher_2 = NetworkTableInstance.getDefault().getStructTopic("correctedPose", Pose2d.struct).publish();
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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules){

        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        if(Utils.isSimulation()) {
            startSimThread();
        }

        pathplanner();

        LimelightHelpers.SetIMUMode("limelight-four", 4);

        robotFieldCentric = false;
        isBooleanTrue = -1;

        publisher = NetworkTableInstance.getDefault().getStructTopic("botpose", Pose2d.struct).publish();
        limelightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightPose", Pose2d.struct).publish();
        publisher_2 = NetworkTableInstance.getDefault().getStructTopic("correctedPose", Pose2d.struct).publish();
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

    public Command c_seedFieldRelativeWithVision() {
    return runOnce(() -> {
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");
        if (mt2.tagCount > 0) {
            // This resets X, Y, and Rotation to the Vision's "Truth"
            this.resetPose(mt2.pose); 
        }
    });
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
            case SLOW:
                return .25;
            case FAST:
                return .75;
            case VERY_FAST:
                return 1;
        }
        return 1;
    }

    /**
     * Assigns a speed to the drivetrain specified by the mode.
     * @param speed as percentage from 0 to 1.
     */
    public void setSpeed(int speed){
        if (m_speed.ordinal() + speed > SPEED.VERY_FAST.ordinal()){
            m_speed = SPEED.SLOW;

        } else if (m_speed.ordinal() + speed < SPEED.SLOW.ordinal()){
            m_speed = SPEED.VERY_FAST;

        } else {
            m_speed = SPEED.values()[m_speed.ordinal() + speed];

        }
        System.out.printf("Updated Speed: %d\r\n", m_speed.ordinal());
    }

    /**
     * Runs a command to supply the speed mode to the drivetrain.
     * @param speedMode as an int from 0 to 2
     * @return the command.
     */
    public Command c_updateSpeed(int speedMode){
        return runOnce(() -> setSpeed(speedMode));
    }
}
