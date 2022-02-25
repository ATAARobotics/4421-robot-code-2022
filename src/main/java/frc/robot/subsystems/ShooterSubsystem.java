package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Climber;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);

    private Climber climber = null;
    
    private double lowSpeed = 0.5;
    private double highCloseSpeed = 0.95;
    private double highFarSpeed = 0.95;

    public ShooterSubsystem(Climber climber) {
        this.climber = climber;
    }

    @Override
    public void periodic() {

    }

    public void shooterLow() {
        shootMotor.set(-1 * lowSpeed);
        climber.armTilt();
    }

    public void shooterHighClose() {
        shootMotor.set(-1 * highCloseSpeed);
        climber.armTilt();
    }

    public void shooterHighFar() {
        shootMotor.set(-1 * highFarSpeed);
        climber.armTilt();
    }

    public void shooterOff() {
        shootMotor.set(0);
    }
}
