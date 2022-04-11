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
    private double tolerance = 0.04;

    private CANSparkMax secondaryMotor = new CANSparkMax(RobotMap.SECONDARY_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder secondaryEncoder = secondaryMotor.getEncoder();
    private SparkMaxPIDController secondaryPID = secondaryMotor.getPIDController();
    
    private double[] lowSpeed = { 2500, 0 };
    private double[] highFarSpeed = { 5000, 4500 };
    private double[] launchpadSpeed = { 4300, 4500 };
    private double[] autoWallSpeed = { 2500, 0 };
    private double[] autoDotSpeed = { 3300, 8000 };
    private double[] autoFourthSpeed = { 3780, 8000 };

    private double mainSetpoint;
    private double secondarySetpoint;

    private double mainError;
    private double secondaryError;

    public ShooterSubsystem(String bus) {
        mainMotor.setInverted(true);
        secondaryMotor.setInverted(true);
        mainPID.setP(0.00045);
        mainPID.setI(0.00001);
        mainPID.setD(0.00003);
        mainPID.setFF(0.00018);

        secondaryPID.setP(0.000043);
        secondaryPID.setI(0.000001);
        secondaryPID.setD(0.0000003);
        secondaryPID.setFF(0.00009);

        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Main Velocity", mainEncoder.getVelocity());
        SmartDashboard.putNumber("Main Setpoint", mainSetpoint);
        SmartDashboard.putNumber("Secondary Velocity", secondaryEncoder.getVelocity());
        SmartDashboard.putNumber("Secondary Setpoint", secondarySetpoint);
    }

    public void shooterTestInit() {
        mainSetpoint = launchpadSpeed[0];
        secondarySetpoint = launchpadSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(8000, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Secondary kP", 0);
        SmartDashboard.putNumber("Secondary kI", 0);        
        SmartDashboard.putNumber("Secondary kFF", 0.0005);   
        SmartDashboard.putNumber("Secondary kD", 0);
    }

    public void shooterTestPeriodic() {
        secondaryPID.setP(SmartDashboard.getNumber("Secondary kP", 0));
        secondaryPID.setI(SmartDashboard.getNumber("Secondary kI", 0));     
        secondaryPID.setD(SmartDashboard.getNumber("Secondary kD", 0));
        secondaryPID.setFF(SmartDashboard.getNumber("Secondary kFF", 0.0005));
    }
    
    public void shooterLow() {
        mainSetpoint = lowSpeed[0];
        secondarySetpoint = lowSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();
    }

    public void shooterHighFar() {
        mainSetpoint = highFarSpeed[0];
        secondarySetpoint = highFarSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();
}

    public void shooterLaunchpad() {
        mainSetpoint = launchpadSpeed[0];
        secondarySetpoint = launchpadSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();
    }
    
    public void shooterAutoFourth() {
        mainSetpoint = autoFourthSpeed[0];
        secondarySetpoint = autoFourthSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();
    }

    public void shooterAutoWall() {
        mainSetpoint = autoWallSpeed[0];
        secondarySetpoint = autoWallSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();

    }
    public void shooterAutoDot() {
        mainSetpoint = autoDotSpeed[0];
        secondarySetpoint = autoDotSpeed[1];
        mainPID.setOutputRange(0, 1);
        secondaryPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setReference(secondarySetpoint, CANSparkMax.ControlType.kVelocity);
        pidReset();
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
        boolean mainOK, secondaryOK;
        mainError = mainSetpoint-mainEncoder.getVelocity();
        secondaryError = secondarySetpoint - secondaryEncoder.getVelocity();
        if (mainSetpoint == 0) {
            mainOK = true;
        } else {
            mainOK = Math.abs(mainError/mainSetpoint) <= tolerance;
        }
        if (secondarySetpoint == 0) {
            secondaryOK = true;
        } else {
            secondaryOK = Math.abs(secondaryError/secondarySetpoint) <= tolerance;
        }
        return mainOK && secondaryOK;
    }
}
