package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);
    
    private double lowSpeed = 0.5;
    private double highCloseSpeed = 0.95;
    private double highFarSpeed = 0.95;

    public ShooterSubsystem() {
    }

    @Override
    public void periodic() {

    }

    public void shooterLow() {
        shootMotor.set(-1 * lowSpeed);
    }

    public void shooterHighClose() {
        shootMotor.set(-1 * highCloseSpeed);
    }

    public void shooterHighFar() {
        shootMotor.set(-1 * highFarSpeed);
    }

    public void shooterOff() {
        shootMotor.set(0);
    }
}
