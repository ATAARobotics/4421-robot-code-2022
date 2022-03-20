package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private DigitalInput elevatorDownDetector = new DigitalInput(6);
    private RelativeEncoder m_elevatorEncoder;
    private double elevatorSpeed = 0.85;
    private double minElevatorMaxEncoderTicks = 100;
    private double midElevatorMaxEncoderTicks = 500;
    private double maxElevatorMaxEncoderTicks = 0;


    public ClimbMotorSubsystem() {
        m_elevatorEncoder = elevator.getEncoder();
        elevator.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Down", climberMin());
        SmartDashboard.putNumber("speed", elevatorSpeed);
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
        if (!climberMin()) {
            elevator.set(-elevatorSpeed);
        } else {
            elevator.set(0.0);
        }
    }

    public void climberStop() {
        elevator.set(0.0);
    }

    public boolean climberMin() {
        return !elevatorDownDetector.get();
    }

    public boolean climberMid() {
        return m_elevatorEncoder.getPosition() >= midElevatorMaxEncoderTicks;
    }

    public boolean climberMax() {
        return m_elevatorEncoder.getPosition() >= maxElevatorMaxEncoderTicks;
    }
}
