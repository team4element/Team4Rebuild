/*
 * This class puts together the commands from the subsystems and assigns them to triggers on the controllers.
 * The auton commands are also defined here.
 */

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutoAim;
import frc.robot.Commands.CombinedShoot;
import frc.robot.Commands.CombinedTapShoot;
import frc.robot.Commands.CornerShot;
import frc.robot.Commands.FreePivot;
import frc.robot.Commands.IntakeFuel;
import frc.robot.Commands.PositionPivot;
import frc.robot.Commands.RetractIntake;
import frc.robot.Commands.TapPivot;
import frc.robot.Commands.TurretManual;
import frc.robot.Commands.TurretToPosition;
import frc.robot.Commands.Auton.IntakeForAuto;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Spindexer;
import frc.robot.Subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here.
  SendableChooser<Command> sendableAuton;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.80).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(1.0).withRotationalDeadband(.9)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentric fcDrive = new SwerveRequest.FieldCentric()
            .withDeadband(1.0).withRotationalDeadband(.9)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  public final CommandSwerveDrivetrain m_drivetrain;
  public final Turret m_turret;
  public final Intake m_intake;
  public final Pivot m_pivot;
  public final Spindexer m_spinster;
  public final Conveyor m_conveyor;
  public final Shooter m_shooter;

  public final AprilTagFieldLayout m_field_layout;

  private boolean initialized;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_field_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_drivetrain = TunerConstants.createDrivetrain(m_field_layout);
    m_turret = new Turret(m_field_layout, m_drivetrain);
    m_shooter = new Shooter(m_drivetrain);
    m_intake = new Intake();
    m_pivot = new Pivot();
    m_spinster = new Spindexer();
    m_conveyor = new Conveyor();

    initialized = false;

    // These are the commands used in auton.
    NamedCommands.registerCommand("Long Shot", new CombinedTapShoot(m_shooter, m_conveyor, m_spinster, m_intake, m_pivot).withTimeout(12));
    NamedCommands.registerCommand("Aim", new AutoAim(m_turret).withTimeout(0.5));
    NamedCommands.registerCommand("Shoot", new CombinedShoot(m_shooter, m_conveyor, m_spinster).withTimeout(3));
    NamedCommands.registerCommand("TurretHuman", new TurretToPosition(m_turret, TurretConstants.rightCornerRotation).withTimeout(0.5));
    NamedCommands.registerCommand("Extend + Intake", new IntakeForAuto(m_intake, m_pivot).withTimeout(IntakeConstants.intakeTimeout));
    NamedCommands.registerCommand("Retract", new PositionPivot(m_intake, m_pivot, PivotConstants.poseForAuto).withTimeout(IntakeConstants.intakeTimeout));
    NamedCommands.registerCommand("Tap Intake", new TapPivot(m_intake, m_pivot).repeatedly().withTimeout(IntakeConstants.intakeTimeout));

    sendableAuton = AutoBuilder.buildAutoChooser();
    sendableAuton.setDefaultOption("Center to Human", AutoBuilder.buildAuto("Center to Human"));
    SmartDashboard.putData("Auto Chooser", sendableAuton);

    configureBindings();
  }

  private void configureBindings() {

    // These are the subsystem default commands.
    m_drivetrain.setDefaultCommand(
     //  Drivetrain will execute this command periodically
     m_drivetrain.applyRequest(() ->
      fcDrive.withVelocityX(ControllerConstants.yTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftY() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
              .withVelocityY(ControllerConstants.xTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftX() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
              .withRotationalRate(ControllerConstants.zRotationModifier.apply(
                -ControllerConstants.driverController.getRightX() * MaxAngularRate * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
      )
    );

    // These are the driver controls:
    ControllerConstants.driverController.rightBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));
    ControllerConstants.driverController.leftBumper().onTrue(m_drivetrain.applyRequest(() ->
        drive.withVelocityX(ControllerConstants.yTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftY() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive forward with negative Y (forward)
             .withVelocityY(ControllerConstants.xTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftX() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive left with negative X (left)
             .withRotationalRate(ControllerConstants.zRotationModifier.apply(
                -ControllerConstants.driverController.getRightX() * MaxAngularRate * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive counterclockwise with negative X (left)
    ));

    ControllerConstants.driverController.start().onTrue(m_drivetrain.c_updateSpeed(1)); // Increases speed by one SPEED Enum
    ControllerConstants.driverController.back().onTrue(m_drivetrain.c_updateSpeed(-1)); //Lowers speed by one SPEED Enum

    ControllerConstants.driverController.povLeft().onTrue(new TurretToPosition(m_turret, TurretConstants.leftCornerRotation)); // Rotates pivot for the left corner shot.
    ControllerConstants.driverController.povRight().onTrue(new TurretToPosition(m_turret, TurretConstants.rightCornerRotation)); // Rotates pivot for the right corner shot.

    // These are the operator controls:
    ControllerConstants.operatorController.y().whileTrue(new CombinedShoot(m_shooter, m_conveyor, m_spinster));
    ControllerConstants.operatorController.b().whileTrue(new TapPivot(m_intake, m_pivot).repeatedly());
    ControllerConstants.operatorController.x().onTrue(new PositionPivot(m_intake, m_pivot, PivotConstants.poseToIntake)); // Lowers pivot to be ready to intake.
    ControllerConstants.operatorController.a().whileTrue(new AutoAim(m_turret));

    // Move turret manually.
    ControllerConstants.operatorController.povRight().whileTrue(new TurretManual(m_turret));
    ControllerConstants.operatorController.povLeft().whileTrue(new TurretManual(m_turret)); 

    // Inverse conveyor systems.
    ControllerConstants.operatorController.start().whileTrue(new CornerShot(m_shooter, m_conveyor, m_spinster, ShooterConstants.cornerSpeed).withTimeout(5)); // Shooting from corner.
    //ControllerConstants.operatorController.povUp().whileTrue(new TurretToPosition(m_turret, 0)); // Returns the turret to it's starting position (forward).
    ControllerConstants.operatorController.povDown().whileTrue(new FreePivot(m_pivot, m_intake, PivotConstants.pivotSpeed));
    ControllerConstants.operatorController.povUp().whileTrue(new FreePivot(m_pivot, m_intake, -PivotConstants.pivotSpeed));

    ControllerConstants.operatorController.leftBumper().whileTrue(new IntakeFuel(m_intake, -IntakeConstants.intakeSpeed)); // Runs the outtake.
    ControllerConstants.operatorController.rightBumper().whileTrue(new IntakeFuel(m_intake, IntakeConstants.intakeSpeed)); // Runs the intake.
    ControllerConstants.operatorController.leftTrigger().whileTrue(new RetractIntake(m_intake, m_pivot, -PivotConstants.pivotSpeed)); // Lowers the pivot of the intake.
    ControllerConstants.operatorController.rightTrigger().whileTrue(new RetractIntake(m_intake, m_pivot, PivotConstants.pivotSpeed)); // Raises the pivot of the intake.
  }

  public void onInit(){
    LimelightHelpers.setCameraPose_RobotSpace(
      "limelight-four",
      VisionConstants.forwardOffsetMeters,
      VisionConstants.sideOffsetMeters,
      VisionConstants.altitudeMeters,
      180,
      VisionConstants.mountedDegree,
      0);
  }

  public void onEnable(Pose2d startLocation) {
    if(!initialized) {
      initialized = true;
      var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

      LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.initialIMUMode);
      LimelightHelpers.SetIMUAssistAlpha(VisionConstants.kLimelightName, 0.001); // Adds a correction on the IMU mode.

      // 1. Determine the Pose: Start with a hardcoded default.
      Pose2d startPose = (alliance == Alliance.Red)
        ? new Pose2d(12.9, 4.0, Rotation2d.fromDegrees(180)) // Robot starting in front of the center of red hub.
        : new Pose2d(3.65, 4.0, Rotation2d.fromDegrees(0)); // Robot starting in front of the center of blue hub.

      // 2. If PathPlanner provided a specific start, override the default.
      if(startLocation != null) {
        startPose = startLocation;
      }

      // 3. APPLY the pose to the hardware (This needs to happen regardless)
      m_turret.resetTurret();
      m_pivot.homePivot();
      m_drivetrain.seedFieldCentric();
      m_drivetrain.resetPose(startPose);

      // 4. Set Perspective: This makes sure "Forward" on the joystick is correct
      m_drivetrain.setOperatorPerspectiveForward(
        alliance == Alliance.Red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)
      );

      System.out.println("Robot Initialized at: " + startPose.toString() + " | Alliance: " + alliance + " | Gyro: " + m_drivetrain.getState().Pose.getRotation().getDegrees() + " | Start Location: " + startLocation);
    }
  }

  /*
   * This resets the turret's position to 0 when the robot disables.
   */
  public void onDisable() {
    LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 1);
    m_turret.setYaw(0);
    initialized = false;
  }

  /*
   * Switches the drivetrain's forward to be relative to the field (toward the opposing alliance).
   */
  public Command c_fieldRelative() {
    return m_drivetrain.applyRequest(() -> fcDrive);
  }

  /**
   * Gets the auton made using Pathplanner which is selected from a drop-down menu.
   * @returns the auton.
   */
   public Command getAutonomousCommand() {  
    return sendableAuton.getSelected();
  }
}
