package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber {
    CANSparkMax elevator = new CANSparkMax(13, MotorType.kBrushless);
    double elevatorSpeed = 0.5;
    double elevatorSpeedChange = 0.1;

    public Climber() {

    }
    public void increaseSpeed() {
        elevatorSpeed += elevatorSpeedChange;
        if (elevatorSpeed > 1) {
            elevatorSpeed = 1;
        }
    }
    
    public void decreaseSpeed() {
        elevatorSpeed -= elevatorSpeedChange;
        if (elevatorSpeed < 0) {
            elevatorSpeed = 0;
        }
    }

    public void climberUp() {
        elevator.set(elevatorSpeed);
    }

    public void climberDown() {
        elevator.set(-elevatorSpeed);
    }
    public void climberDirectionEnable(int speed) {
        elevator.set(elevatorSpeed*speed);
    }
}
