package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);
    private double shootSpeed = 1;

    public ShooterSubsystem() {
        SmartDashboard.putNumber("ShootSpeed", shootSpeed);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      shootSpeed = SmartDashboard.getNumber("ShootSpeed", 1);
    }

    public void shooterPercentage() {
        shootMotor.set(-1*shootSpeed);
    }

    public void shooterOff() {
        shootMotor.set(0);
    }
}
