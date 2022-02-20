package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private RelativeEncoder m_elevatorEncoder;
    private double elevatorSpeed = 0.5;
    private int elevatorMaxEncoderTicks = 0;

    public ClimbMotorSubsystem() {
        m_elevatorEncoder = elevator.getEncoder();
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
        return m_elevatorEncoder.getPosition() <= 100;
    }

    public boolean climberMax() {
        return m_elevatorEncoder.getPosition() >= elevatorMaxEncoderTicks;
    }
}
