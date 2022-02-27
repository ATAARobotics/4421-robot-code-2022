package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0);
    private BetterJoystick gunnerStick = new BetterJoystick(1);

    private boolean intakeIsFront = RobotMap.INTAKE_STARTS_FRONT;

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean toggleFieldOriented;
    private boolean switchCameras;
    private int elevatorDirection;
    private boolean toggleClimbArm;
    private boolean toggleIntake;
    private boolean toggleShooterPercent;
    private boolean toggleShooterPID;
    
    public JoystickButton climbMotorUp;
    public JoystickButton climbMotorDown;
    public JoystickButton climbArm;
    public JoystickButton climbSlow;
    public JoystickButton climbFast;
    public JoystickButton autoClimbSwing;
    public JoystickButton autoClimbUp;
    public JoystickButton autoClimbTwo;
    public JoystickButton intake;
    public JoystickButton shootLow;
    public JoystickButton shootHighClose;
    public JoystickButton shootHighFar;

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
        intake = driveStick.getWPIJoystickButton("Intake");
        shootLow = driveStick.getWPIJoystickButton("ShootLow");
        shootHighClose = driveStick.getWPIJoystickButton("ShootHighClose");
        shootHighFar = driveStick.getWPIJoystickButton("ShootHighFar");
        climbMotorUp = gunnerStick.getWPIJoystickButton("ElevatorUp");
        climbMotorDown = gunnerStick.getWPIJoystickButton("ElevatorDown");
        climbArm = gunnerStick.getWPIJoystickButton("ToggleClimbArm");
        climbSlow = gunnerStick.getWPIJoystickButton("ClimbSlow");
        climbFast = gunnerStick.getWPIJoystickButton("ClimbFast");
        //autoClimbSwing = gunnerStick.getWPIJoystickButton("AutoClimbSwing");
        //autoClimbUp = gunnerStick.getWPIJoystickButton("AutoClimbUp");
        //autoClimbTwo = gunnerStick.getWPIJoystickButton("AutoClimbTwo");
    }

    //Periodic function to update controller input
    public void checkInputs() {

        if (driveStick.getButton("SwitchFronts")) {
            intakeIsFront = !intakeIsFront;
        }

        xVelocity = driveStick.getAnalog("XVelocity");
        yVelocity = driveStick.getAnalog("YVelocity");
        rotationVelocity = driveStick.getAnalog("RotationVelocity");

        //Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < RobotMap.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < RobotMap.JOY_DEAD_ZONE) { rotationVelocity = 0; }

        if (!intakeIsFront) {
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
        switchCameras = driveStick.getButton("SwitchCameras");
    }

    //Getter functions for controls
    public double getXVelocity() {
        return xVelocity;
    }
    public double getYVelocity() {
        return yVelocity;
    }
    public double getRotationVelocity() {
        return rotationVelocity;
    }
    public boolean getToggleFieldOriented() {
        return toggleFieldOriented;
    }
    public boolean getSwitchCameras() {
        return switchCameras;
    }
    public int getElevatorDirection() {
        return elevatorDirection;
    }
    
    public boolean getToggleIntake() {
        return toggleIntake;
    }

    public boolean getToggleShootPercent() {
        return toggleShooterPercent;
    }

    public boolean getToggleShootPID() {
        return toggleShooterPID;
    }
    public boolean getToggleClimbArm() {
        return toggleClimbArm;
    }
}
