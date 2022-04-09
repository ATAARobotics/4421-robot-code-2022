package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(RobotMap.MAIN_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder mainEncoder = mainMotor.getEncoder();
    private SparkMaxPIDController mainPID = mainMotor.getPIDController();
    private double tolerance = 100;

    private PWMVictorSPX secondaryMotor = new PWMVictorSPX(RobotMap.SECONDARY_SHOOT_MOTOR_PORT);
    private CANCoder secondaryEncoder;
    private PIDController secondaryPID = new PIDController(0.001, 0.0275, 0.0004);
    
    //Overridden by teleop and auto
    private double[] lowSpeed = { 0, 0 };
    private double[] highFarSpeed = { 0, 0 };
    private double[] launchpadSpeed = { 0, 0 };

    private double secondaryVelocityDivided;
    private double mainSetpoint;
    private double secondarySetpoint;

    private int curSpeedLevel = 0;

    private double mainError;
    private double secondaryError;

    public ShooterSubsystem(String bus) {
        mainMotor.setInverted(true);
        //TODO delete the smart dashboards once the pid and setpoints are configured each getnumber should be replaced with the second parameter only
        SmartDashboard.putNumber("MainPID highfar setpoint", 4500); 
        SmartDashboard.putNumber("SecondaryPID highfar setpoint", 120);

        SmartDashboard.putNumber("MainPID P", 0.0001);
        SmartDashboard.putNumber("MainPID I", 0.000000001);
        SmartDashboard.putNumber("MainPID D", 0.000000001);
        SmartDashboard.putNumber("MainPID FF", 0.0001705);

        SmartDashboard.putNumber("SecondaryPID P", 0.001);
        SmartDashboard.putNumber("SecondaryPID I", 0.0275);
        SmartDashboard.putNumber("SecondaryPID D", 0.0004);


        mainPID.setP(SmartDashboard.getNumber("MainPID P", 0.0001)); //replace with second parameter only after configuation
        mainPID.setI(SmartDashboard.getNumber("MainPID I", 0.000000001));
        mainPID.setD(SmartDashboard.getNumber("MainPID D", 0.000000001));
        mainPID.setFF(SmartDashboard.getNumber("MainPID FF", 0.0001705));

        secondaryPID.setP(SmartDashboard.getNumber("SecondaryPID P", 0.001));
        secondaryPID.setI(SmartDashboard.getNumber("SecondaryPID I", 0.0275));
        secondaryPID.setD(SmartDashboard.getNumber("SecondaryPID D", 0.0004));

        mainPID.setOutputRange(0, 1);
        
        CANCoder secondaryEncoder = new CANCoder(RobotMap.SECONDARY_SHOOT_ENCODER_ID, bus);
        this.secondaryEncoder = secondaryEncoder;
        secondaryPID.setSetpoint(launchpadSpeed[1]);
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Main Velocity", mainEncoder.getVelocity());
        SmartDashboard.putNumber("Main Setpoint", mainSetpoint);
        SmartDashboard.putNumber("Secondary Velocity", secondaryVelocityDivided);
        SmartDashboard.putNumber("Secondary Setpoint", secondarySetpoint);
    }
    public void shooterPeriodic() {
        secondaryVelocityDivided = secondaryEncoder.getVelocity() / -100;
        secondarySetpoint = secondaryPID.getSetpoint();

        //Secondary motor
        if (secondarySetpoint == 0.0) {
            secondaryMotor.set(0);
            secondaryPID.reset();
        } else {
            secondaryMotor.set(secondaryPID.calculate(secondaryVelocityDivided));
        }
    }

    public void autonomousMode() {
        lowSpeed = new double[] { 95, 0 };
        highFarSpeed = new double[] { 118, 3000 };
        launchpadSpeed = new double[] { 139, 4000 };
    }

    public void teleopMode() {
        lowSpeed = new double[] { 2500, 0 }; //TODO: UPDATE, number up to 5700
        highFarSpeed = new double[] {4500, 118 }; //TODO: UPDATE up to 5700
        launchpadSpeed = new double[] { 4500, 120 }; //TODO: UPDATE up to 5700
    }

    public void shooterLow() {
        mainSetpoint = lowSpeed[0];
        mainPID.setReference(lowSpeed[0], CANSparkMax.ControlType.kVelocity);
        mainPID.setOutputRange(0, 1);
        secondaryPID.setSetpoint(lowSpeed[1]);
        if (curSpeedLevel != 0) {
            pidReset();
            curSpeedLevel = 0;
        }
    }

    public void shooterHighFar() {
        mainSetpoint = SmartDashboard.getNumber("MainPID highfar setpoint", 4000);
        mainPID.setReference(SmartDashboard.getNumber("MainPID highfar setpoint", 4000), CANSparkMax.ControlType.kVelocity);
        mainPID.setOutputRange(0, 1);
        secondaryPID.setSetpoint(SmartDashboard.getNumber("SecondaryPID highfar setpoint", 120));
        if (curSpeedLevel != 1) {
            pidReset();
            curSpeedLevel = 1;
        }
}

    public void shooterLaunchpad() {
         mainSetpoint = SmartDashboard.getNumber("MainPID lanchpad setpoint", 4500); //TODO replace with launchpadspeed[0]
        mainPID.setReference(SmartDashboard.getNumber("MainPID lanchpad setpoint", 4500), CANSparkMax.ControlType.kVelocity);
        mainPID.setOutputRange(0, 1);
        secondaryPID.setSetpoint(SmartDashboard.getNumber("SecondaryPID lanchpad setpoint", 120));
        if (curSpeedLevel != 2) {
            pidReset();
            curSpeedLevel = 2;
        }
    }
    
    public void shooterAutoFourth() {
        mainSetpoint = 135;
        mainPID.setReference(135, CANSparkMax.ControlType.kVelocity);
        mainPID.setOutputRange(0, 1);
        secondaryPID.setSetpoint(145);
        if (curSpeedLevel != 3) {
            pidReset();
            curSpeedLevel = 3;
        }
    }

    public void shooterOff() {
        mainPID.setReference(0.0, CANSparkMax.ControlType.kVelocity);
        mainPID.setOutputRange(0, 0);
        secondaryPID.setSetpoint(0.0);
    }

    public boolean atSetpoint() {
        return (Math.abs(mainEncoder.getVelocity() - mainSetpoint) < tolerance) && secondaryPID.atSetpoint();
    }

    public void pidReset() {
        mainPID.setIAccum(0);
        secondaryPID.reset();
    }

    public boolean nearSetpoint() {
        mainError = mainSetpoint-mainEncoder.getVelocity();
        secondaryError = secondaryPID.getSetpoint() - (secondaryVelocityDivided);
        return (Math.abs(mainError) <= 100);
    }
}
