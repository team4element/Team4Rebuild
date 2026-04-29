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
import frc.robot.Commands.Scoring.AutoAimMove;
import frc.robot.Commands.Scoring.CombinedPassMove;
import frc.robot.Commands.Scoring.CombinedShootMove;
import frc.robot.Commands.TransferFuel;
import frc.robot.Commands.Scoring.TurretManual;
import frc.robot.Commands.Scoring.TurretToPosition;
import frc.robot.Commands.Auton.CombinedTapShootAuto;
import frc.robot.Commands.Auton.IntakeForAuto;
import frc.robot.Commands.Intake.FreePivot;
import frc.robot.Commands.Intake.HoldPivot;
import frc.robot.Commands.Intake.IntakeFuel;
import frc.robot.Commands.Intake.PositionPivot;
import frc.robot.Commands.Intake.RetractIntake;
import frc.robot.Commands.Intake.TapPivot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SpinsterConstants;
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

// Uncomment the line below to use for demos
// import frc.robot.Commands.Scoring.ShootingDemo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.
  SendableChooser<Command> sendableAuton;

  /* 
   * This controls the speed of the drivetrain's turning and drive. 
   * To alter the MaxSpeed, add a multiplier, to alter the MaxAngularRate, change the magnitude.
   */ 
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.95).in(RadiansPerSecond); 

  /* 
   * This makes the drivetrain drive relative to the field
   * meaning forward on the joystick will always make the robot drive toward the opposite alliance. 
   */ 
  private final SwerveRequest.FieldCentric fcDrive = new SwerveRequest.FieldCentric()
            .withDeadband(1.0).withRotationalDeadband(.9)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

            
  // --- CREATING OBJECTS ---
  public final CommandSwerveDrivetrain m_drivetrain;
  public final Turret m_turret;
  public final Intake m_intake;
  public final Pivot m_pivot;
  public final Spindexer m_spinster;
  public final Conveyor m_conveyor;
  public final Shooter m_shooter;

  public final AprilTagFieldLayout m_field_layout;

  private boolean initialized;
  private boolean match_started;

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
    NamedCommands.registerCommand("Long Shot", new CombinedTapShootAuto(m_field_layout, m_drivetrain, m_shooter, m_turret, m_conveyor, m_spinster, m_intake, m_pivot).withTimeout(12));
    NamedCommands.registerCommand("Aim", new AutoAimMove(m_turret, m_field_layout).withTimeout(0.4));
    NamedCommands.registerCommand("Shoot", new CombinedTapShootAuto(m_field_layout, m_drivetrain, m_shooter, m_turret, m_conveyor, m_spinster, m_intake, m_pivot).withTimeout(2));
    NamedCommands.registerCommand("TurretHuman", new TurretToPosition(m_turret, TurretConstants.rightCornerRotation).withTimeout(0.5));
    NamedCommands.registerCommand("Extend + Intake", new IntakeForAuto(m_intake, m_pivot).withTimeout(IntakeConstants.intakeTimeout));
    NamedCommands.registerCommand("ExtendSlow", new IntakeForAuto(m_intake, m_pivot).withTimeout(IntakeConstants.intakeTimeout*1.5));
    NamedCommands.registerCommand("Retract", new PositionPivot(m_intake, m_pivot, PivotConstants.poseForAuto).withTimeout(1.5));
    NamedCommands.registerCommand("Up", new PositionPivot(m_intake, m_pivot, PivotConstants.upwardAuto).withTimeout(1));
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

    m_pivot.setDefaultCommand(new HoldPivot(m_pivot));

    // These are the driver controls:
    ControllerConstants.driverController.leftTrigger().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

    ControllerConstants.driverController.rightBumper().onTrue(m_drivetrain.c_updateSpeed(1)); // Increases speed by one SPEED Enum
    ControllerConstants.driverController.leftBumper().onTrue(m_drivetrain.c_updateSpeed(-1)); //Lowers speed by one SPEED Enum

    // These are the operator controls:
    ControllerConstants.operatorController.b().whileTrue(new TapPivot(m_intake, m_pivot).repeatedly());
    ControllerConstants.operatorController.x().whileTrue(new TransferFuel(m_spinster, m_conveyor, SpinsterConstants.spinsterSpeed, -ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.a().whileTrue(new AutoAimMove(m_turret, m_field_layout));
    ControllerConstants.operatorController.y().whileTrue(new CombinedShootMove(m_turret, m_shooter, m_conveyor, m_spinster, m_drivetrain, m_field_layout));

    // Move turret manually.
    ControllerConstants.operatorController.povRight().whileTrue(new TurretManual(m_turret));
    ControllerConstants.operatorController.povLeft().whileTrue(new TurretManual(m_turret));

    // Inverse conveyor systems.
    ControllerConstants.operatorController.start().whileTrue(new CombinedPassMove(m_turret, m_shooter, m_conveyor, m_spinster, m_drivetrain)); 
    ControllerConstants.operatorController.back().whileTrue(new TurretToPosition(m_turret, 0));

    // Controls the arm without using limits.
    ControllerConstants.operatorController.povDown().whileTrue(new FreePivot(m_pivot, m_intake, PivotConstants.pivotSpeed));
    ControllerConstants.operatorController.povUp().whileTrue(new FreePivot(m_pivot, m_intake, -PivotConstants.pivotSpeed));

    ControllerConstants.operatorController.leftBumper().whileTrue(new IntakeFuel(m_intake, -IntakeConstants.intakeSpeed)); // Runs the outtake.
    ControllerConstants.operatorController.rightBumper().whileTrue(new IntakeFuel(m_intake, IntakeConstants.intakeSpeed)); // Runs the intake.

    ControllerConstants.operatorController.leftTrigger().whileTrue(new RetractIntake(m_intake, m_pivot, -PivotConstants.pivotSpeed)); // Lowers the pivot of the intake and outakes.
    ControllerConstants.operatorController.rightTrigger().whileTrue(new RetractIntake(m_intake, m_pivot, PivotConstants.pivotSpeed)); // Raises the pivot of the intake and intakes.
  
    // These are used for demoing the robot (shoot at set speed).
    // ControllerConstants.operatorController.b().whileTrue(new ShootingDemo(100, m_shooter, m_conveyor, m_spinster));  // This is a far shot.
    // ControllerConstants.operatorController.start().whileTrue(new ShootingDemo(70, m_shooter, m_conveyor, m_spinster));
    // ControllerConstants.operatorController.y().whileTrue(new ShootingDemo(50, m_shooter, m_conveyor, m_spinster)); // This is a short shot
  
  }

  public void onEnable(Pose2d startLocation) {
    if (!initialized) {
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 4);

        var allianceEntry = DriverStation.getAlliance();

        // If we don't know the alliance yet, we might want to skip one loop
        // but for now, let's just make the Red check very explicit.
        boolean isRed = allianceEntry.isPresent() && allianceEntry.get() == Alliance.Red;

        Pose2d startPose;
        // Default to the PathPlanner start if it exists, regardless of alliance
        if (startLocation != null) {
            startPose = startLocation;
        } else {
            // Only use hardcoded poses if PathPlanner didn't give us one
            if (isRed) {
                startPose = new Pose2d(12.9, 4.0, Rotation2d.fromDegrees(180));
            } else {
                startPose = new Pose2d(3.65, 4.0, Rotation2d.fromDegrees(0));
            }
        }

        if (DriverStation.isAutonomous()) {
            match_started = true;
        }

        // Apply hardware changes
        m_turret.resetTurret();
        m_pivot.homePivot();
        m_drivetrain.seedFieldCentric();
        m_drivetrain.resetPose(startPose);

        m_drivetrain.setOperatorPerspectiveForward(
            isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)
        );

        initialized = true;
        System.out.println("Initialized! Red: " + isRed + " Pose: " + startPose);
    }
  }

  public void onDisable() {
    LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 1);

    m_turret.setYaw(0);
    // m_pivot.onDisable();
    
    if (!DriverStation.isFMSAttached() && !match_started) {
      initialized = false;
    }
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
