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
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
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
    private boolean doRejectUpdate;
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

    private Field2d field;

    // This is used to get the robot's position on the field using the limelight data. It stands for MegaTag2. 
    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();
    StructPublisher<Pose2d> publisher; 
    StructPublisher<Pose2d> limelightPublisher;
    StructPublisher<Pose2d> publisher_2;

    // This is indicates how much 'trust' or reliability is put into the limelight's data for mt2. Smaller numbers mean more trust. 
  //  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  //  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

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

    // This gets each module location in meters from the center of the robot.
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        new Translation2d(0.283, 0.283), // The location of the front left module from center of robot in meters.
        new Translation2d(0.283, -0.283), // The location of the front right module from center of robot in meters.
        new Translation2d(-0.283, 0.283), // The location of the back left module from center of robot in meters.
        new Translation2d(-0.283, 0.283) // The location of the back right module from center of robot in meters.
    );

    // This is used to build the robot's location.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        kKinematics,
        TunerConstants.m_pigeon.getRotation2d(),
        this.getState().ModulePositions,
        new Pose2d()
    );

    /**
     * @return the pigeon's rotation.
     */
    public Rotation2d getGyroAngle(){
        return TunerConstants.m_pigeon.getRotation2d();
    }

    //public SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(kKinematics, getGyroAngle(), getModuleLocations(), m_odometry);

    public void setVisionPose(){
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four");
        var odometryPose = this.getState().Pose;

        publisher.set(odometryPose);
        limelightPublisher.set(mt1.pose);

        doRejectUpdate = false;

        if(mt1 == null || mt1.tagCount == 0){
         doRejectUpdate = true;
        }


       // We start at the turret center and "swing" out by the camera radius
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
            if(mt1.rawFiducials[0].ambiguity > 0.7){
                doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 4){
                 doRejectUpdate = true;
            }
        }

        if(mt1.pose.getTranslation().getDistance(
            this.getState().Pose.getTranslation()) > 3.0){
            doRejectUpdate = true;
        }

        if(!doRejectUpdate){
            double xyStdDev = 0.2 / mt1.tagCount * mt1.avgTagDist;
            double thetaStdDev = mt1.tagCount >= 2 ? 0.2 : 9999;

            this.addVisionMeasurement(
            mt1.pose,
            Utils.fpgaToCurrentTime(mt1.timestampSeconds), // REMEMBER THIS TIME UNIT CONVERSION, OTHERWISE CTRE SWERVE WILL DISREGARD THE POSE
            VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            );
        }else {
            field.setRobotPose(odometryPose);
        }
    }

    /**
     * Inverses the boolean, which starts as false, when the left or right bumpers are triggered (switch from robot centric to field centric).
     * @return weather or not robot is driving robot centric or field centric.
     */
    public boolean isFieldCentric(){
        if(ControllerConstants.driverController.leftBumper().getAsBoolean() || ControllerConstants.driverController.rightBumper().getAsBoolean()){
            isBooleanTrue*=(-1);
        }

        if(isBooleanTrue < 0){
            robotFieldCentric = false;
        }else if(isBooleanTrue > 0){
            robotFieldCentric = true;
        }
        return robotFieldCentric;
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
        
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        field.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four").pose);
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
        
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        field.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four").pose);
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

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        field.setRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four").pose);
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
        System.out.println(getGyroAngle());

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

    public void setVisionMeasurementStdDevs(Vector<N3> stdDevs) {
        super.setVisionMeasurementStdDevs(stdDevs);
    }

    @Override
    public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
        var trust = .5;
        var use_gyro_heading = 999999;
        super.addVisionMeasurement(robotPose, timestamp, VecBuilder.fill(trust, trust, use_gyro_heading));
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

    // Convert to field-relative using the robot's current rotation
    public ChassisSpeeds getFieldRelativeVelocity() {
        var state = this.getState();
        ChassisSpeeds robotSpeeds = kKinematics.toChassisSpeeds(state.ModuleStates);
    
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, state.Pose.getRotation());
    }
}
