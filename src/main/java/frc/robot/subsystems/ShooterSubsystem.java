package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(RobotMap.MAIN_SHOOT_MOTOR, MotorType.kBrushless);
    private CANCoder mainEncoder = new CANCoder(RobotMap.MAIN_SHOOT_ENCODER);
    private PIDController mainPID = new PIDController(0.015, 0.02, 0.001);

    private VictorSPX secondaryMotor = new VictorSPX(RobotMap.SECONDARY_SHOOT_MOTOR);
    private CANCoder secondaryEncoder = new CANCoder(RobotMap.SECONDARY_SHOOT_ENCODER);
    private PIDController secondaryPID = new PIDController(0.001, 0.0275, 0.0004);
    
    //Overridden by teleop and auto
    private double[] lowSpeed = { 0, 0 };
    private double[] highCloseSpeed = { 0, 0 };
    private double[] highFarSpeed = { 0, 0 };
    private double[] launchpadSpeed = { 0, 0 };

    private boolean reversing = false;

    public ShooterSubsystem() {
        mainMotor.setInverted(true);
        secondaryMotor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        if (!reversing) {
            SmartDashboard.putNumber("Main Velocity", mainEncoder.getVelocity() / 10000);
            SmartDashboard.putNumber("Main Setpoint", mainPID.getSetpoint());
            SmartDashboard.putNumber("Secondary Velocity", secondaryEncoder.getVelocity() / -100);
            SmartDashboard.putNumber("Secondary Setpoint", secondaryPID.getSetpoint());
            if (mainPID.getSetpoint() == 0.0) {
                mainMotor.set(0);
                mainPID.reset();
            } else {
                mainMotor.set(mainPID.calculate(mainEncoder.getVelocity() / 10000));
            }
            if (secondaryPID.getSetpoint() == 0.0) {
                secondaryMotor.set(ControlMode.PercentOutput, 0);
                secondaryPID.reset();
            } else {
                secondaryMotor.set(ControlMode.PercentOutput, secondaryPID.calculate(secondaryEncoder.getVelocity() / -100));
            }
        }
        reversing = false;
    }

    public void autonomousMode() {
        lowSpeed = new double[] { 95, 0 };
        highFarSpeed = new double[] { 118, 118 };
        launchpadSpeed = new double[] { 139, 139 };
    }

    public void teleopMode() {
        lowSpeed = new double[] { 95, 0 };
        highCloseSpeed = new double[] { 0, 0 };
        highFarSpeed = new double[] { 119, 120 }; //If we want to go to the dots where the balls are set, we can go to 125, 125 or add a new preset
        launchpadSpeed = new double[] { 125, 145 };
    }

    public void shooterReverse() {
        reversing = true;
        mainMotor.set(-0.2);
        secondaryMotor.set(ControlMode.PercentOutput, -0.2);
    }

    public void shooterLow() {
        mainPID.setSetpoint(lowSpeed[0]);
        secondaryPID.setSetpoint(lowSpeed[1]);
    }

    public void shooterHighClose() {
        mainPID.setSetpoint(highCloseSpeed[0]);
        secondaryPID.setSetpoint(highCloseSpeed[1]);
    }

    public void shooterHighFar() {
        mainPID.setSetpoint(highFarSpeed[0]);
        secondaryPID.setSetpoint(highFarSpeed[1]);
}

    public void shooterLaunchpad() {
        mainPID.setSetpoint(launchpadSpeed[0]);
        secondaryPID.setSetpoint(launchpadSpeed[1]);
    }

    public void shooterOff() {
        mainPID.setSetpoint(0.0);
        secondaryPID.setSetpoint(0.0);
    }
}
