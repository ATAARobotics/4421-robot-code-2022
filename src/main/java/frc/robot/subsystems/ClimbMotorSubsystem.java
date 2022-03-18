package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private CANCoder m_elevatorEncoder = new CANCoder(RobotMap.CLIMB_ENCODER);
    private double elevatorSpeed = 0.85;
    private double minElevatorEncoderTicks = 50;
    private double midElevatorEncoderTicks = 1500;
    private double maxElevatorEncoderTicks = 3600;

    private double elevatorTolerance = 50;


    public ClimbMotorSubsystem() {
        elevator.setInverted(true);
        m_elevatorEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber", m_elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("Max", climberMax());
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
        return m_elevatorEncoder.getPosition() <= minElevatorEncoderTicks + elevatorTolerance;
    }

    public boolean climberMid() {
        return m_elevatorEncoder.getPosition() >= midElevatorEncoderTicks + elevatorTolerance;
    }

    public boolean climberMax() {
        return m_elevatorEncoder.getPosition() >= maxElevatorEncoderTicks + elevatorTolerance;
    }
}
