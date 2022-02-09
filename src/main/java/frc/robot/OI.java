package frc.robot;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

class OI {

    private XboxController driveStick = new XboxController(0);
    private XboxController gunnerStick = new XboxController(1);
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
    public final int[] intake = {1, Button.kLeftBumper.value};
    public final int[] shooter = {1, Button.kA.value};

    public OI() {
        
    }

    //Periodic function to update controller input
    public void checkInputs() {
        xVelocity = driveStick.getLeftX();
        yVelocity = driveStick.getLeftY();
        rotationVelocity = driveStick.getRightX();
        decreaseElevatorSpeed = driveStick.getXButtonPressed();
        increaseElevatorSpeed = driveStick.getBButtonPressed();
        toggleClimbArm = driveStick.getRightBumperReleased();

        if(driveStick.getAButton() == driveStick.getYButton()) {
            elevatorDirection = 0;
        }
        else if(driveStick.getAButton()) {
            elevatorDirection = -1;
        }
        else if(driveStick.getYButton()) {
            elevatorDirection = 1;
        }

        //Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < RobotMap.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < RobotMap.JOY_DEAD_ZONE) { rotationVelocity = 0; }

        toggleFieldOriented = driveStick.getXButtonPressed();
        toggleCamera = driveStick.getRightBumperPressed();
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
