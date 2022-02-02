package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_ARM[0], RobotMap.CLIMB_ARM[1]);
    private boolean armTilted = false;
    private double elevatorSpeed = 0.5;
    private double elevatorSpeedChange = 0.1;

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
        SmartDashboard.putNumber("climber speed", elevatorSpeed);
    }

    public void toggleArm() {
        armTilted = !armTilted;
        if (armTilted) {
            armTilt();
        } else {
            armVertical();
        }
    }

    public void armTilt() {
        arm.set(Value.kReverse);
    }
    public void armVertical() {
        arm.set(Value.kForward);
    }
}
