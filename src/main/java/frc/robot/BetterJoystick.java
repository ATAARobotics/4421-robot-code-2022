package frc.robot;

import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

public class BetterJoystick {

    private XboxController controller = null;
    private Properties bindings = null;
    
    public BetterJoystick(int port) {
        controller = new XboxController(port);
    }

    public boolean getButton(String action) {
        if (bindings == null) {
            DriverStation.reportError("Button bindings not configured yet!", false);
            return false;
        }

        String button = bindings.getProperty(action);

        if (button == "None") {
            return false;
        }

        String[] buttonInfo = button.split(" - ");

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

    public double getAnalog(String action) {
        if (bindings == null) {
            DriverStation.reportError("Button bindings not configured yet!", false);
            return 0.0;
        }

        String axis = bindings.getProperty(action);

        if (axis == "None") {
            return 0.0;
        }

        int axisID;
        switch (axis) {
            case "LeftX":
                axisID = 0;
                break;
            
            case "LeftY":
                axisID = 1;
                break;

            case "RightX":
                axisID = 4;
                break;

            case "RightY":
                axisID = 5;
                break;

            case "LeftTrigger":
                axisID = 2;
                break;

            case "RightTrigger":
                axisID = 3;
                break;

            default:
                DriverStation.reportError("There is no analog trigger with the name " + axis, false);
                return 0.0;
        }

        return controller.getRawAxis(axisID);
    }

    public void configureBindings(Properties bindings) {
        this.bindings = bindings;
    }

    public XboxController getController() {
        return controller;
    }
}
