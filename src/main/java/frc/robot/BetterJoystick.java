package frc.robot;

import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A better interface for Xbox controllers
 */
public class BetterJoystick {

    private XboxController controller = null;
    private Properties bindings = null;
    
    /**
     * Creates a new BetterJoystick using the given joystick port.
     * 
     * @param port The joystick port that the Xbox controller to use is plugged into
     */
    public BetterJoystick(int port) {
        controller = new XboxController(port);
    }

    /**
     * Check whether a given button has been activated
     * 
     * @param action The action name (not the button name) to check for - these are configured by the bindings provided
     */
    public boolean getButton(String action) {
        if (bindings == null) {
            DriverStation.reportError("Button bindings not configured yet!", false);
            return false;
        }

        String button = bindings.getProperty(action, "None");

        if (button.equals("None")) {
            return false;
        }

        String[] buttonInfo = button.split("-");

        if (buttonInfo.length != 2) {
            DriverStation.reportError("There does not appear to be exactly two arguments in the button " + button, false);
            return false;
        }

        int buttonID;
        switch (buttonInfo[0]) {
            case "X":
                buttonID = 3;
                break;
            
            case "Y":
                buttonID = 4;
                break;

            case "A":
                buttonID = 1;
                break;
            
            case "B":
                buttonID = 2;
                break;

            case "LeftBumper":
                buttonID = 5;
                break;

            case "RightBumper":
                buttonID = 6;
                break;

            case "Start":
                buttonID = 8;
                break;

            case "Back":
                buttonID = 7;
                break;

            case "LeftJoystick":
                buttonID = 9;
                break;

            case "RightJoystick":
                buttonID = 10;
                break;
        
            default:
                DriverStation.reportError("There is no button with the name " + buttonInfo[0], false);
                return false;
        }
        switch (buttonInfo[1]) {
            case "Held":
                return controller.getRawButton(buttonID);

            case "Pressed":
                return controller.getRawButtonPressed(buttonID);

            case "Released":
                return controller.getRawButtonReleased(buttonID);

            default:
                DriverStation.reportError("There is no button query type with the name " + buttonInfo[1], false);
                return false;
        }
    }

    /**
     * Get the value of a given analog trigger
     * 
     * @param action The action name (not the trigger name) to get - these are configured by the bindings provided
     */
    public double getAnalog(String action) {
        if (bindings == null) {
            DriverStation.reportError("Button bindings not configured yet!", false);
            return 0.0;
        }

        String trigger = bindings.getProperty(action, "None");

        if (trigger == "None") {
            return 0.0;
        }

        int triggerID;
        switch (trigger) {
            case "LeftX":
                triggerID = 0;
                break;
            
            case "LeftY":
                triggerID = 1;
                break;

            case "RightX":
                triggerID = 4;
                break;

            case "RightY":
                triggerID = 5;
                break;

            case "LeftTrigger":
                triggerID = 2;
                break;

            case "RightTrigger":
                triggerID = 3;
                break;

            default:
                DriverStation.reportError("There is no analog trigger with the name " + trigger, false);
                return 0.0;
        }

        return controller.getRawAxis(triggerID);
    }

    public JoystickButton getWPIJoystickButton(String action) {
        if (bindings == null) {
            DriverStation.reportError("Button bindings not configured yet!", false);
            return new JoystickButton(controller, 100);
        }

        String button = bindings.getProperty(action);

        int buttonID;
        switch (button) {
            case "X":
                buttonID = 3;
                break;
            
            case "Y":
                buttonID = 4;
                break;

            case "A":
                buttonID = 1;
                break;
            
            case "B":
                buttonID = 2;
                break;

            case "LeftBumper":
                buttonID = 5;
                break;

            case "RightBumper":
                buttonID = 6;
                break;

            case "Start":
                buttonID = 8;
                break;

            case "Back":
                buttonID = 7;
                break;

            case "LeftJoystick":
                buttonID = 9;
                break;

            case "RightJoystick":
                buttonID = 10;
                break;
        
            default:
                DriverStation.reportError("There is no button with the name " + button, false);
                return new JoystickButton(controller, 100);
        }

        return new JoystickButton(controller, buttonID);
    }

    /**
     * Set the intensity of the joystick rumble
     * 
     * @param rumble The intensity of the rumble (0 to 1, 0 being off and 1 being max)
     */
    public void setRumble(double rumble) {
        controller.setRumble(RumbleType.kLeftRumble, rumble);
        controller.setRumble(RumbleType.kRightRumble, rumble);
    }

    /**
     * Configure the button bindings using a Properties object
     * 
     * @param bindings The Properties object to read in the button bindings from
     */
    public void configureBindings(Properties bindings) {
        this.bindings = bindings;
    }

    /**
     * Get the base XboxController class, if you need it for some reason.
     * This should probably only be used if you are mixing together the command-based robot template with the iterative robot template.
     */
    public XboxController getController() {
        return controller;
    }

}
