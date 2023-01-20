package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0, 1);
    private BetterJoystick gunnerStick = new BetterJoystick(1, 0);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean toggleFieldOriented;
    private int elevatorDirection;
    private boolean toggleClimbArm;
    private boolean toggleIntake;
    private boolean toggleShooterPercent;
    private boolean toggleShooterPID;
    
    private boolean toggleDriveTag;
    
    public JoystickButton climbMotorUp;
    public JoystickButton climbMotorDown;
    public JoystickButton climbArm;
    public JoystickButton climbSlow;
    public JoystickButton climbFast;
    public JoystickButton intake;
    public JoystickButton cancelShooterRev;
    public Trigger shootLow;
    public Trigger shootHighFar;
    public Trigger shootLaunchpad;
    //public Trigger abortVisionAlign;

    public JoystickButton driveTag;

    public JoystickButton aimRight;
    public JoystickButton aimLeft;
    private double speed;

    public OI() {
        //Configure the button bindings
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

        //Set up command-based stuff


        driveTag = gunnerStick.getWPIJoystickButton("DriveTag");
    }

    //Periodic function to update controller input


    // public boolean aimRight() {
        
    //     DriverStation.reportWarning("Aiming Right", false);
    //     return aimLeft.getAsBoolean();
    // }


}
