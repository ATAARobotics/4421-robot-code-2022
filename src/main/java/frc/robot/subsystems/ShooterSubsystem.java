package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(Constants.MAIN_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder mainEncoder = mainMotor.getEncoder();
    private SparkMaxPIDController mainPID = mainMotor.getPIDController();
    private double tolerance = 0.02;

    private CANSparkMax secondaryMotor = new CANSparkMax(Constants.SECONDARY_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private CANCoder secondaryEncoder;
    private PIDController secondaryPID = new PIDController(0.000079, 0.00035, 0.000001);

    private double[] lowSpeed = { 2500, 0 };
    private double[] highFarSpeed = { 3800, 2000 };
    // THESE WORK FROM THE RING OF DOTS - PEOPLE MIGHT WANT THESE BACK private
    // double[] highFarSpeed = { 3750, 90 };
    private double[] launchpadSpeed = { 3800, 120 };
    private double[] autoSpeed = { 3700, 95 };
    private double[][] distanceSpeed = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{3700,105},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},};

    private double mainSetpoint;
    private double secondarySetpoint;
    private double secondaryRPM;
    private double mainError;
    private double secondaryError;

    private int mode = -1;

    public ShooterSubsystem(String bus) {
        mainMotor.setInverted(true);
        secondaryMotor.setInverted(true);
        mainPID.setP(0.000455);
        mainPID.setI(0.00001);
        mainPID.setD(0.00003);
        mainPID.setFF(0.000195);

        mainPID.setOutputRange(0, 1);

        secondaryEncoder = new CANCoder(Constants.SECONDARY_SHOOT_ENCODER_ID, "rio");
    }

    @Override
    public void periodic() {
        secondarySetpoint = secondaryPID.getSetpoint();
        secondaryRPM = secondaryEncoder.getVelocity()/6;
        // Secondary motor
        if (secondarySetpoint == 0.0) {
            secondaryMotor.set(0);
            secondaryPID.reset();
        } else {
            secondaryMotor.set(secondaryPID.calculate(secondaryRPM));
        }
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Main Velocity", mainEncoder.getVelocity());
        SmartDashboard.putNumber("Main Setpoint", mainSetpoint);
        SmartDashboard.putNumber("Secondary Velocity", secondaryRPM);
        SmartDashboard.putNumber("Secondary Setpoint", secondarySetpoint);
    }

    public void shooterLow() {
        mainSetpoint = lowSpeed[0];
        secondarySetpoint = lowSpeed[1];

        mainPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);
        if (mode != 0) {
            pidReset();
            mode = 0;
        }
        fixMain();
    }

    public void shooterHighFar() {
        mainSetpoint = highFarSpeed[0];
        secondarySetpoint = highFarSpeed[1];

        mainPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);
        if (mode != 1) {
            pidReset();
            mode = 1;
        }
        fixMain();
    }

    public void shooterDistance(double dist){
        int index = (int)(((Math.round(dist * 2)/2)/0.5)-1);

        System.out.println(distanceSpeed[index][0]);
        System.out.println(index);

        mainSetpoint = distanceSpeed[index][0];
        secondarySetpoint = distanceSpeed[index][1];

        mainPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);

        if (mode != 1) {
            pidReset();
            mode = 1;
        }
        fixMain();
    }
    
    public void shooterLaunchpad() {
        mainSetpoint = launchpadSpeed[0];
        secondarySetpoint = launchpadSpeed[1];

        mainPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);
        if (mode != 2) {
            pidReset();
            mode = 2;
        }
        fixMain();
    }

    public void shooterAuto() {
        mainSetpoint = autoSpeed[0];
        secondarySetpoint = autoSpeed[1];
        mainPID.setOutputRange(0, 1);
        mainPID.setReference(mainSetpoint, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);
        if (mode != 5) {
            pidReset();
            mode = 5;
        }
        fixMain();
    }

    public void shooterOff() {
        secondarySetpoint = 0;
        mainPID.setOutputRange(0, 0);
        mainPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        secondaryPID.setSetpoint(secondarySetpoint);
        mode = -1;
    }

    public void pidReset() {

        // secondaryPID.reset();
    }

    public void fixMain() {
        mainPID.setIAccum(0);
    }

    public double getSpeedPrimary() {
        return mainEncoder.getVelocity();
    }

    public double getSpeedSecondary() {
        return secondaryRPM;
    }

    public boolean nearSetpoint() {
        boolean mainOK, secondaryOK;
        mainError = mainSetpoint - mainEncoder.getVelocity();
        secondaryError = secondarySetpoint - secondaryRPM;
        if (mainSetpoint == 0) {
            mainOK = true;
        } else {
            mainOK = Math.abs(mainError / mainSetpoint) <= tolerance;
        }
        if (secondarySetpoint == 0) {
            secondaryOK = true;
        } else {
            secondaryOK = Math.abs(secondaryError / secondarySetpoint) <= tolerance;
        }
        return mainOK;
    }
}
