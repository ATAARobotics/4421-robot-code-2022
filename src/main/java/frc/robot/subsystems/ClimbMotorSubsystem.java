package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax climbMotor = new CANSparkMax(RobotMap.CLIMB_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder m_elevatorEncoder = climbMotor.getEncoder();
    private DigitalInput elevatorDownDetector = new DigitalInput(7);
    private double climbMotorSpeed = 0.85;

    private double midElevatorEncoderTicks = 35;
    private double maxElevatorEncoderTicks = 150;

    public ClimbMotorSubsystem() {
        climbMotor.setInverted(true);
        climbMotor.setIdleMode(IdleMode.kBrake);

        resetElevatorEncoder();
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Elevator Encoder", m_elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("Climber Mid", atMid());
        SmartDashboard.putBoolean("Climber Max", atMax());
        SmartDashboard.putBoolean("Climber Min", atMin());
        SmartDashboard.putNumber("speed", climbMotorSpeed);
    }

    public void setSlowSpeed() {
        setSpeed(1);
    }

    public void setNormalSpeed() {
        setSpeed(2);
    }

    public void setMaxSpeed() {
        setSpeed(3);
    }

    public void setSpeed(int speedLevel) {
        switch (speedLevel) {
            case 1:
                climbMotorSpeed = 0.3;
                break;

            case 2:
                climbMotorSpeed = 0.85;
                break;

            case 3:
                climbMotorSpeed = 1.0;
                break;

            default:
                DriverStation.reportError(speedLevel + " is not a valid speed level for the climber!", false);
                break;
        }
    }

    public void up() {
        climbMotor.set(climbMotorSpeed);
    }

    public void down() {
        if (!atMin()) {
            climbMotor.set(-climbMotorSpeed);
        } else {
            climbMotor.set(0.0);
        }
    }

    public void stop() {
        climbMotor.set(0.0);
    }

    public void resetElevatorEncoder() {
        m_elevatorEncoder.setPosition(0);
    }

    public boolean atMin() {
        return !elevatorDownDetector.get();
    }

    public boolean atMid() {
        return m_elevatorEncoder.getPosition() >= midElevatorEncoderTicks;
    }

    public boolean atMax() {
        return m_elevatorEncoder.getPosition() >= maxElevatorEncoderTicks;
    }

    public void slowRate() {
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0); // Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0); // Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0); // Disable
    }
}
