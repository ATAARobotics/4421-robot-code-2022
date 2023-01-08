package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0, 1);
    private BetterJoystick gunnerStick = new BetterJoystick(1, 0);

    public OI() {
        // Configure the button bindings
        try (InputStream input = new FileInputStream("/home/lvuser/deploy/bindings.properties")) {
            Properties bindings = new Properties();

            bindings.load(input);

            driveStick.configureBindings(bindings);
            gunnerStick.configureBindings(bindings);

            input.close();
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Button bindings file not found!", false);
        } catch (IOException e) {
            DriverStation.reportError("IOException on button binding file", false);
        }

    }

    public void rumbleGunnerOn() {
        gunnerStick.setRumble(1);
    }

    public void rumbleGunnerOff() {
        gunnerStick.setRumble(0);
    }

    public void checkInputs() {
    }

}
