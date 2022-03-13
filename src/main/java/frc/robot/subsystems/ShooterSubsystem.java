package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(RobotMap.MAIN_SHOOT_MOTOR, MotorType.kBrushless);
    private CANCoder mainEncoder = new CANCoder(RobotMap.MAIN_SHOOT_ENCODER);
    private PIDController mainPID = new PIDController(0.015, 0.02, 0.001);
    
    //Overridden by teleop and auto
    private double[] lowSpeed = { 0, 0 };
    private double[] highCloseSpeed = { 0, 0 };
    private double[] highFarSpeed = { 0, 0 };

    private boolean reversing = false;

    public ShooterSubsystem() {
        mainMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        if (!reversing) {
            if (mainPID.getSetpoint() == 0.0) {
                mainMotor.set(0);
                mainPID.reset();
            } else {
                mainMotor.set(mainPID.calculate(mainEncoder.getVelocity() / 10000));
            }
        }
        reversing = false;
    }

    public void autonomousMode() {
        lowSpeed = new double[] { 94.2, 0 };
        highFarSpeed = new double[] { 155, 0 };
    }

    public void teleopMode() {
        lowSpeed = new double[] { 94.2, 0 };
        highCloseSpeed = new double[] { 165, 0 };
        highFarSpeed = new double[] { 180, 0 };
    }

    public void shooterReverse() {
        reversing = true;
        mainMotor.set(-0.2);
    }

    public void shooterLow() {
        mainPID.setSetpoint(lowSpeed[0]);
    }

    public void shooterHighClose() {
        mainPID.setSetpoint(highCloseSpeed[0]);
    }

    public void shooterHighFar() {
        mainPID.setSetpoint(highFarSpeed[0]);
    }

    public void shooterOff() {
        mainPID.setSetpoint(0.0);
    }
}
