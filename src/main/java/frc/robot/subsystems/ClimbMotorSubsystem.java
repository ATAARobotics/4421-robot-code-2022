package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private RelativeEncoder m_elevatorEncoder;
    private double elevatorSpeed = 0.7;
    private double minElevatorMaxEncoderTicks = 100;
    private double midElevatorMaxEncoderTicks = 500;
    private double maxElevatorMaxEncoderTicks = 0;


    public ClimbMotorSubsystem() {
        m_elevatorEncoder = elevator.getEncoder();
    }

    public void climberSlowSpeed() {
        setSpeed(1);
    }
    public void climberNormalSpeed() {
        setSpeed(2);
    }
    public void climberMaxSpeed() {
        setSpeed(3);
    }

    public void setSpeed(int speedLevel) {
        switch (speedLevel) {
            case 1:
                elevatorSpeed = 0.3;
                break;
            
            case 2:
                elevatorSpeed = 0.85;
                break;

            case 3:
                elevatorSpeed = 1.0;
                break;
        
            default:
                DriverStation.reportError(speedLevel + " is not a valid speed level for the climber!", false);
                break;
        }
    }

    public void climberUp() {
        elevator.set(elevatorSpeed);
    }

    public void climberDown() {
        elevator.set(-elevatorSpeed);
    }

    public void climberStop() {
        elevator.set(0);
        elevator.stopMotor();
        elevator.disable();

    }

    public boolean climberMin() {
        return m_elevatorEncoder.getPosition() <= minElevatorMaxEncoderTicks;
    }

    public boolean climberMid() {
        return m_elevatorEncoder.getPosition() >= midElevatorMaxEncoderTicks;
    }

    public boolean climberMax() {
        return m_elevatorEncoder.getPosition() >= maxElevatorMaxEncoderTicks;
    }
}
