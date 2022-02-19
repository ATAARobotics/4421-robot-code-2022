package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax elevator = new CANSparkMax(RobotMap.CLIMB_MOTOR, MotorType.kBrushless);
    private double elevatorSpeed = 0.5;

    public ClimbMotorSubsystem() {

    }

    public void climberUp() {
        elevator.set(elevatorSpeed);
    }

    public void climberDown() {
        elevator.set(-elevatorSpeed);
    }
}
