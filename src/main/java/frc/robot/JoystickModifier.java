// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * We want to create a custom class which extends a joystick's reading and
 * applies a "smoothing"
 * Function to it.
 * 
 * In order to do this, we need a toggle between different options.
 * 
 * Options:
 * 1. Deadband --> parameter which controls the deadband zone.
 * 2. Slew Rate --> parameter which controls how fast the joystick can ramp up
 * to full speed.
 * 3. Quadratic --> No parameters needed.
 * 
 */

public class JoystickModifier {
    // Member Variables
    // Saving variables for later
    String option = "Linear";

    // Need to manually check for changes in the slew rate
    double lastInput = 0;
    double currentSlewRate = 1;

    LiveDoubleBinding deadbandEntry;
    LiveDoubleBinding slewRateEntry;

    double deadband = 0;

    SlewRateLimiter slewRateLimiter = new SlewRateLimiter(currentSlewRate);

    // Constructor
    public JoystickModifier(String name) {
        System.out.println("JoystickModifier constructor");
        SendableChooser<String> chooser = new SendableChooser<>();

        chooser.setDefaultOption("Linear", "Linear");
        chooser.addOption("Quadratic", "Quadratic");
        chooser.addOption("Slew", "Slew");

        chooser.onChange((newOption) -> {
            System.out.println("You chose: " + newOption);
            option = newOption;
        });

        ShuffleboardTab tab = Shuffleboard.getTab("Joystick");

        tab.add(name + "/JoystickOptions", chooser);

        deadbandEntry = new LiveDoubleBinding("Joystick", name + "/xTranslationModifier/DeadbandValue", 0.0);
        slewRateEntry = new LiveDoubleBinding("Joystick", name + "/xTranslationModifier/SlewRateValue", 1.0);
    }

    /***
     * Modifies the joystick input based on the input mode
     * 
     * @param input Input from the joystick
     * @return The modified input
     */

    public double apply(double input) {
        // Apply Deadband
        double currentDeadband = deadbandEntry.getDouble();

        input = MathUtil.applyDeadband(input, currentDeadband);

        // Check if the slew rate has changed
        if (slewRateEntry.getDouble(currentSlewRate) != currentSlewRate) {
            currentSlewRate = slewRateEntry.getDouble(currentSlewRate);
            slewRateLimiter = new SlewRateLimiter(currentSlewRate);
            slewRateLimiter.reset(lastInput);
        }

        lastInput = input;

        double modifiedInput = input;

        if (option.equals("Linear")) {
            modifiedInput = input;
        }

        if (option.equals("Quadratic")) {
            modifiedInput = input * input;
        }

        if (option.equals("Slew")) {
            modifiedInput = slewRateLimiter.calculate(input);
        }

        return modifiedInput;
    }
}

