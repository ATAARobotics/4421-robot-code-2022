package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);

    public ShooterSubsystem() {

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void shooterPercentage() {
        shootMotor.set(-1);
    }

    public void shooterOff() {
        shootMotor.set(0);
    }
}
