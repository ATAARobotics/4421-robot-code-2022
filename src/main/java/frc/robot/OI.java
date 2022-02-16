package frc.robot;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0);
    private BetterJoystick gunnerStick = new BetterJoystick(1);
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean toggleFieldOriented;
    private boolean toggleCamera;
    private int elevatorDirection;
    private boolean toggleClimbArm;
    private boolean decreaseElevatorSpeed;
    private boolean increaseElevatorSpeed;
    private boolean toggleIntake;
    private boolean toggleShooterPercent;
    private boolean toggleShooterPID;
    public final JoystickButton intake = new JoystickButton(gunnerStick.getController(), Button.kB.value);
    public final JoystickButton shooter = new JoystickButton(gunnerStick.getController(), Button.kA.value);
    public final JoystickButton magazine = new JoystickButton(gunnerStick.getController(), Button.kY.value);

    public OI() {
        //Configure the button bindings
        try (InputStream input = new FileInputStream("./bindings.properties")) {
            Properties bindings = new Properties();

            bindings.load(input);

            driveStick.configureBindings(bindings);
            gunnerStick.configureBindings(bindings);

            input.close();
        } catch (IOException e) {

        }
    }

    //Periodic function to update controller input
    public void checkInputs() {
        xVelocity = driveStick.getAnalog("XVelocity");
        yVelocity = driveStick.getAnalog("YVelocity");
        rotationVelocity = driveStick.getAnalog("RotationVelocity");
        decreaseElevatorSpeed = driveStick.getButton("DecreaseElevatorSpeed");
        increaseElevatorSpeed = driveStick.getButton("IncreaseElevatorSpeed");
        toggleClimbArm = driveStick.getButton("ToggleClimbArm");

        if(driveStick.getButton("ElevatorDown") == driveStick.getButton("ElevatorUp")) {
            elevatorDirection = 0;
        }
        else if(driveStick.getButton("ElevatorDown")) {
            elevatorDirection = -1;
        }
        else if(driveStick.getButton("ElevatorUp")) {
            elevatorDirection = 1;
        }

        //Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < RobotMap.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < RobotMap.JOY_DEAD_ZONE) { rotationVelocity = 0; }

        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
        toggleCamera = driveStick.getButton("RightBumper-Pressed");
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
    public boolean getToggleCamera() {
        return toggleCamera;
    }
    public int getElevatorDirection() {
        return elevatorDirection;
    }
    public boolean getElevatorSpeedDecreased() {
        return decreaseElevatorSpeed;
    }
    public boolean getElevatorSpeedIncreased() {
        return increaseElevatorSpeed;
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
