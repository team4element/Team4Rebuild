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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.CombinedShoot;
import frc.robot.Commands.ConveyToTurret;
import frc.robot.Commands.FindApriltag;
import frc.robot.Commands.IntakeFuel;
import frc.robot.Commands.PositionPivot;
import frc.robot.Commands.RetractIntake;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.TapPivot;
import frc.robot.Commands.TransferFuel;
import frc.robot.Commands.TurretManual;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Spinster;
import frc.robot.Subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here.
  SendableChooser<Command> sendableAuton;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*0.5; // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.40).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  // Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(.6).withRotationalDeadband(.6)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.FieldCentric fcDrive = new SwerveRequest.FieldCentric()
            .withDeadband(.6).withRotationalDeadband(.6)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  public final CommandSwerveDrivetrain m_drivetrain;
  public final Turret m_turret;
  //public final Climb m_climb;
  public final Intake m_intake;
  public final Spinster m_spinster;
  public final Conveyor m_conveyor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain = TunerConstants.createDrivetrain();
    // Use the drivetrain in the turret.
    m_turret = new Turret(m_drivetrain);
    //m_climb = new Climb();
    m_intake = new Intake();
    m_spinster = new Spinster();
    m_conveyor = new Conveyor();

    // Configure the trigger bindings
    NamedCommands.registerCommand("Shoot", new FindApriltag(m_turret).withTimeout(TurretConstants.shooterTimeout));
    //NamedCommands.registerCommand("Climb", new ManualClimbDown(m_climb, ClimbConstants.climbSpeed).withTimeout(ClimbConstants.climbTimeout));
    NamedCommands.registerCommand("Extend + Intake", new IntakeFuel(m_intake, IntakeConstants.rollerSpeed).withTimeout(IntakeConstants.intakeTimeout));
    NamedCommands.registerCommand("Retract", new RetractIntake(m_intake, IntakeConstants.linearPivotSpeed).withTimeout(IntakeConstants.intakeTimeout));
    NamedCommands.registerCommand("Transfer",new ConveyToTurret(m_conveyor, 0.5).withTimeout(ConveyorConstants.conveyorTimeout));
    NamedCommands.registerCommand("Aim", new FindApriltag(m_turret).withTimeout(TurretConstants.aimTimeout));

    sendableAuton = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", sendableAuton);

    configureBindings();
  }

  private void configureBindings() {

    // These are the subsystem default commands.
    m_drivetrain.setDefaultCommand(
     //  Drivetrain will execute this command periodically
      m_drivetrain.applyRequest(() ->
        drive.withVelocityX(ControllerConstants.yTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftY() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive forward with negative Y (forward)
             .withVelocityY(ControllerConstants.xTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftX() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive left with negative X (left)
             .withRotationalRate(ControllerConstants.zRotationModifier.apply(
                -ControllerConstants.driverController.getRightX() * MaxAngularRate * m_drivetrain.speedToDouble(m_drivetrain.m_speed))) // Drive counterclockwise with negative X (left)
      )
    );

    // These are the driver controls: 
    ControllerConstants.driverController.rightBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));
    ControllerConstants.driverController.leftBumper().onTrue(m_drivetrain.applyRequest(() ->
      fcDrive.withVelocityX(ControllerConstants.yTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftY() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
              .withVelocityY(ControllerConstants.xTranslationModifier.apply(
                -ControllerConstants.driverController.getLeftX() * MaxSpeed * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
              .withRotationalRate(ControllerConstants.zRotationModifier.apply(
                -ControllerConstants.driverController.getRightX() * MaxAngularRate * m_drivetrain.speedToDouble(m_drivetrain.m_speed)))
    ));
    
    // These are the operator controls:
    ControllerConstants.operatorController.povRight().whileTrue(new TurretManual(m_turret));
    ControllerConstants.operatorController.povLeft().whileTrue(new TurretManual(m_turret));

    //ControllerConstants.operatorController.start().onTrue(new PositionPivot(m_intake).withTimeout(0.8));
    ControllerConstants.operatorController.back().onTrue(new TapPivot(m_intake, 0.1));

    ControllerConstants.operatorController.y().whileTrue(new Shoot(m_turret));
    ControllerConstants.operatorController.x().whileTrue(new TransferFuel(m_spinster, m_conveyor, 1, -1));
    //ControllerConstants.operatorController.y().whileTrue(new TrackWhileMove(m_turret, m_drivetrain));
    ControllerConstants.operatorController.a().whileTrue(new FindApriltag(m_turret));
    // ControllerConstants.operatorController.b().whileTrue(new TurretToPosition(m_turret, 45));
    ControllerConstants.operatorController.b().whileTrue(new CombinedShoot(m_turret, m_conveyor, m_spinster));

    ControllerConstants.operatorController.leftBumper().whileTrue(new IntakeFuel(m_intake, 0.5));
    ControllerConstants.operatorController.rightBumper().whileTrue(new IntakeFuel(m_intake, -0.5));
    ControllerConstants.operatorController.leftTrigger().whileTrue(new RetractIntake(m_intake, 0.1));
    ControllerConstants.operatorController.rightTrigger().whileTrue(new RetractIntake(m_intake, 0.1));
  }

  /*
   * This runs in initialize on auton to set the climb and turret's starting position.
   */
  public void onEnable(){
    m_intake.resetPivot(m_intake.m_leftPivot);
    m_intake.resetPivot(m_intake.m_rightPivot);
    m_turret.resetTurret();
  }

  public void onDisable(){
    m_turret.returnToStartPosition();
  }

  /*
   * Switches the drivetrain's forward to be relative to the field (toward the opposing alliance).
   */
  public Command c_fieldRelative(){
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
