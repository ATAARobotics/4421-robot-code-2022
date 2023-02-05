package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class OI {

    public BetterJoystick driveStick = new BetterJoystick(0, 1);
    private BetterJoystick rotationStick = new BetterJoystick(1, 1);
    private BetterJoystick gunnerStick = new BetterJoystick(2, 0);

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private int elevatorDirection;
    private boolean toggleClimbArm;
    private boolean toggleIntake;
    private boolean toggleShooterPercent;
    private boolean toggleShooterPID;
    private boolean toggleFieldOriented;

    public Trigger climbMotorUp;
    public Trigger climbMotorDown;
    public JoystickButton climbArm;
    public JoystickButton climbSlow;
    public JoystickButton abortAutoClimb;
    public JoystickButton intake;
    public JoystickButton autoClimb;
    public Trigger shootLow;
    public JoystickButton shootHighFar;
    public JoystickButton shootLaunchpad;
    public Trigger abortVisionAlign;
    // public JoystickButton aimRight;
    public JoystickButton driveTag;
    public JoystickButton aimLeft;
    private double speed;

    public JoystickButton driveStraight;

    public OI() {
        // Configure the button bindings
        try (InputStream input = new FileInputStream("/home/lvuser/deploy/bindings.properties")) {
            Properties bindings = new Properties();

            bindings.load(input);

            driveStick.configureBindings(bindings);
            rotationStick.configureBindings(bindings);
            gunnerStick.configureBindings(bindings);

            input.close();
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Button bindings file not found!", false);
        } catch (IOException e) {
            DriverStation.reportError("IOException on button binding file", false);
        }

        // Set up command-based stuff
        intake = driveStick.getWPIJoystickButton("Intake");
        shootLow = gunnerStick.getDPadTrigger("ShootLow");
        shootHighFar = gunnerStick.getWPIJoystickButton("ShootHighFar");
        shootLaunchpad = gunnerStick.getWPIJoystickButton("ShootLaunchpad");
        abortVisionAlign = gunnerStick.getDPadTrigger("AbortVisionAlign");
        climbMotorUp = gunnerStick.getDPadTrigger("ElevatorUp");
        climbMotorDown = gunnerStick.getDPadTrigger("ElevatorDown");
        climbArm = gunnerStick.getWPIJoystickButton("ToggleClimbArm");
        climbSlow = gunnerStick.getWPIJoystickButton("ClimbSlow");
        abortAutoClimb = gunnerStick.getWPIJoystickButton("AbortAutoClimb");
        // aimRight = gunnerStick.getWPIJoystickButton("AimRight");
        driveTag = gunnerStick.getWPIJoystickButton("DriveTag");
        aimLeft = gunnerStick.getWPIJoystickButton("AimLeft");
        autoClimb = gunnerStick.getWPIJoystickButton("AutoClimb");
        driveStraight = driveStick.getWPIJoystickButton("driveStraight");
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

    public boolean aimLeft() {

        DriverStation.reportWarning("Aiming Left", false);
        return aimLeft.getAsBoolean();
    }

    public boolean aimRight() {

        DriverStation.reportWarning("Aiming Right", false);
        return aimLeft.getAsBoolean();
    }

    public void rumbleGunnerOn() {
        gunnerStick.setRumble(1);
    }

    public void rumbleGunnerOff() {
        gunnerStick.setRumble(0);
    }

    public void checkInputs() {
        xVelocity = driveStick.getAnalog("XVelocity");
        yVelocity = driveStick.getAnalog("YVelocity");
        //rotationVelocity = rotationStick.getAnalog("XVelocity");
        rotationVelocity = driveStick.getAnalog("RotationVelocity");

        speed = -driveStick.getAnalog("Speed");

        // Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < Constants.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < Constants.JOY_DEAD_ZONE) {
            rotationVelocity = 0;
        }

        xVelocity = Math.signum(xVelocity) * Math.abs(Math.pow(xVelocity, Constants.JOYSTICK_SENSITIVITY));
        yVelocity = Math.signum(yVelocity) * Math.abs(Math.pow(yVelocity, Constants.JOYSTICK_SENSITIVITY));
        rotationVelocity = Math.signum(rotationVelocity)
                * Math.abs(Math.pow(rotationVelocity, Constants.TURNING_SENSITIVITY));
        
        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
    }

    // Getter functions for controls
    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getSpeed() {
        return speed;
    }

    public double getRotationVelocity() {
        return rotationVelocity;
    }

    public boolean getToggleFieldOriented() {
        return toggleFieldOriented;
    }
}
