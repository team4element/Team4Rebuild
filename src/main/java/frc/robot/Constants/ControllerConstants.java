package frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.JoystickModifier;

public final class ControllerConstants {

    public static final CommandXboxController driverController = new CommandXboxController(0) ;
    public static final CommandXboxController operatorController = new CommandXboxController(1);

    public static final JoystickModifier yTranslationModifier = new JoystickModifier("yTranslationModifier");
    public static final JoystickModifier xTranslationModifier = new JoystickModifier("xTranslationModifier");
    public static final JoystickModifier zRotationModifier = new JoystickModifier("zRotationModifier");  

    public static final double controllerDeadband = 1.0;
    public static final double controllerRotationalDeadband = 0.9;
}
