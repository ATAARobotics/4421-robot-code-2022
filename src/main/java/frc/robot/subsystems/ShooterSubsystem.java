package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(RobotMap.MAIN_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder mainEncoder = mainMotor.getEncoder();
    private SparkMaxPIDController mainPID = mainMotor.getPIDController();
    private double tolerance = 0.02;

    private CANSparkMax secondaryMotor = new CANSparkMax(RobotMap.SECONDARY_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder secondaryEncoder = secondaryMotor.getEncoder();
    private SparkMaxPIDController secondaryPID = secondaryMotor.getPIDController();
    
    private double[] lowSpeed = { 2500, 0 };
    private double[] highFarSpeed = { 3300, 120 };
    private double[] launchpadSpeed = { 3780, 120 };
    private double[] autoWallSpeed = { 2500, 0 };
    private double[] autoDotSpeed = { 3300, 120 };
    private double[] autoFourthSpeed = {3780, 120};

    private double secondaryVelocityDivided;
    private double mainSetpoint;
    private double secondarySetpoint;

    private int curSpeedLevel = 0;

    private double mainError;
    private double secondaryError;

    public ShooterSubsystem(String bus) {
        mainMotor.setInverted(true);
        mainPID.setP(0.0001); //replace with second parameter only after configuation
        mainPID.setI(0.0000001);
        mainPID.setD(0.000000001);
        mainPID.setFF(0.0001705);

        secondaryPID.setP(0.003);
        secondaryPID.setI(0.0275);
        secondaryPID.setD(0.0000001);
        secondaryPID.setFF(0);

        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Main Velocity", mainEncoder.getVelocity());
        SmartDashboard.putNumber("Main Setpoint", mainSetpoint);
        SmartDashboard.putNumber("Secondary Velocity", secondaryVelocityDivided);
        SmartDashboard.putNumber("Secondary Setpoint", secondarySetpoint);
    }
    
    public void shooterLow() {
        mainSetpoint = lowSpeed[0];
        secondarySetpoint = lowSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 0) {
            pidReset();
            curSpeedLevel = 0;
        }
    }

    public void shooterHighFar() {
        mainSetpoint = highFarSpeed[0];
        secondarySetpoint = highFarSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 1) {
            pidReset();
            curSpeedLevel = 1;
        }
}

    public void shooterLaunchpad() {
        mainSetpoint = launchpadSpeed[0];
        secondarySetpoint = launchpadSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 2) {
            pidReset();
            curSpeedLevel = 2;
        }
    }
    
    public void shooterAutoFourth() {
        mainSetpoint = autoFourthSpeed[0];
        secondarySetpoint = autoFourthSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 3) {
            pidReset();
            curSpeedLevel = 3;
        }
    }

    public void shooterAutoWall() {
        mainSetpoint = autoWallSpeed[0];
        secondarySetpoint = autoWallSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 4) {
            pidReset();
            curSpeedLevel = 4;
        }
    }
    public void shooterAutoDot() {
        mainSetpoint = autoDotSpeed[0];
        secondarySetpoint = autoDotSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        if (curSpeedLevel != 5) {
            pidReset();
            curSpeedLevel = 5;
        }
    }

    public void shooterOff() {
        mainPID.setOutputRange(0, 0);
        secondaryPID.setOutputRange(0, 0);
        mainPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        
    }

    public void pidReset() {
        mainPID.setIAccum(0);
        secondaryPID.setIAccum(0);
    }

    public boolean nearSetpoint() {
        mainError = mainSetpoint-mainEncoder.getVelocity();
        secondaryError = secondarySetpoint - secondaryEncoder.getVelocity();
        return (Math.abs(mainError/mainSetpoint) <= tolerance && Math.abs(secondaryError/secondarySetpoint) <= tolerance);
    }
}
