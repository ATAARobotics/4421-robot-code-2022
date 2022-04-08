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
    
    public JoystickButton climbMotorUp;
    public JoystickButton climbMotorDown;
    public JoystickButton climbArm;
    public JoystickButton climbSlow;
    public JoystickButton climbFast;
    public JoystickButton intake;
    public Trigger shootLow;
    public Trigger shootHighClose;
    public Trigger shootHighFar;
    public Trigger shootLaunchpad;
    public JoystickButton reverseBalls;
    public JoystickButton aimRight;
    public JoystickButton aimLeft;

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
        intake = driveStick.getWPIJoystickButton("IntakeDriver");
        shootLow = driveStick.getWPIJoystickButton("ShootLow");
        shootHighClose = driveStick.getWPIJoystickButton("ShootHighClose");
        shootHighFar = driveStick.getWPIJoystickButton("ShootHighFar").or(new Trigger(() -> gunnerStick.getDPad("ShootHighFarGunner")));
        shootLaunchpad = driveStick.getWPIJoystickButton("ShootLaunchpad").or(new Trigger(() -> gunnerStick.getDPad("ShootLaunchpadGunner")));
        reverseBalls = gunnerStick.getWPIJoystickButton("ReverseBalls");
        climbMotorUp = gunnerStick.getWPIJoystickButton("ElevatorUp");
        climbMotorDown = gunnerStick.getWPIJoystickButton("ElevatorDown");
        climbArm = gunnerStick.getWPIJoystickButton("ToggleClimbArm");
        climbSlow = gunnerStick.getWPIJoystickButton("ClimbSlow");
        climbFast = gunnerStick.getWPIJoystickButton("ClimbFast");
        aimRight = gunnerStick.getWPIJoystickButton("AimRight");
        aimLeft = gunnerStick.getWPIJoystickButton("AimLeft");
    }

    //Periodic function to update controller input
    public void checkInputs() {
        xVelocity = driveStick.getAnalog("XVelocity");
        yVelocity = driveStick.getAnalog("YVelocity");
        rotationVelocity = driveStick.getAnalog("RotationVelocity");

        //Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < RobotMap.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < RobotMap.JOY_DEAD_ZONE) { rotationVelocity = 0; }

        xVelocity = Math.signum(xVelocity) * Math.abs(Math.pow(xVelocity, RobotMap.JOYSTICK_SENSITIVITY));
        yVelocity = Math.signum(yVelocity) * Math.abs(Math.pow(yVelocity, RobotMap.JOYSTICK_SENSITIVITY));
        rotationVelocity = Math.signum(rotationVelocity) * Math.abs(Math.pow(rotationVelocity, RobotMap.TURNING_SENSITIVITY));

        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
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
}
