package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);
    private CANCoder shootEncoder = new CANCoder(RobotMap.SHOOT_ENCODER);
    private PIDController shooterPID = new PIDController(0.015, 0.02, 0.001);
    
    //Overridden by teleop and auto
    private double lowSpeed = 0;
    private double highCloseSpeed = 0;
    private double highFarSpeed = 0;

    private boolean reversing = false;

    public ShooterSubsystem() {
        shootMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        if (!reversing) {
            SmartDashboard.putNumber("Shoot Velocity", shootEncoder.getVelocity() / 10000);
            SmartDashboard.putNumber("Shoot Setpoint", shooterPID.getSetpoint());
            if (shooterPID.getSetpoint() == 0.0) {
                shootMotor.set(0);
                shooterPID.reset();
            } else {
                shootMotor.set(shooterPID.calculate(shootEncoder.getVelocity() / 10000));
            }
        }
        reversing = false;
    }

    public void autonomousMode() {
        lowSpeed = 94.2;
        highFarSpeed = 155;
    }

    public void teleopMode() {
        lowSpeed = 94.2;
        highCloseSpeed = 165;
        highFarSpeed = 180;
    }

    public void shooterReverse() {
        reversing = true;
        shootMotor.set(-0.2);
    }

    public void shooterLow() {
        shooterPID.setSetpoint(lowSpeed);
    }

    public void shooterHighClose() {
        shooterPID.setSetpoint(highCloseSpeed);
    }

    public void shooterHighFar() {
        shooterPID.setSetpoint(highFarSpeed);
    }

    public void shooterOff() {
        shooterPID.setSetpoint(0.0);
    }
}
