package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax mainMotor = new CANSparkMax(RobotMap.MAIN_SHOOT_MOTOR_ID, MotorType.kBrushless);
    private CANCoder mainEncoder = new CANCoder(RobotMap.MAIN_SHOOT_ENCODER_ID);
    private PIDController mainPID = new PIDController(0.015, 0.02, 0.001);

    private VictorSPX secondaryMotor = new VictorSPX(RobotMap.SECONDARY_SHOOT_MOTOR_ID);
    private CANCoder secondaryEncoder = new CANCoder(RobotMap.SECONDARY_SHOOT_ENCODER_ID);
    private PIDController secondaryPID = new PIDController(0.001, 0.0275, 0.0004);
    
    //Overridden by teleop and auto
    private double[] lowSpeed = { 0, 0 };
    private double[] highCloseSpeed = { 0, 0 };
    private double[] highFarSpeed = { 0, 0 };
    private double[] launchpadSpeed = { 0, 0 };

    private double mainVelocityDivided;
    private double secondaryVelocityDivided;
    private double mainSetpoint;
    private double secondarySetpoint;

    private boolean reversing = false;
    private double mainError;
    private double secondaryError;

    public ShooterSubsystem() {
        mainMotor.setInverted(true);
        secondaryMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void diagnostic() {
        SmartDashboard.putNumber("Main Velocity", mainVelocityDivided);
        SmartDashboard.putNumber("Main Setpoint", mainSetpoint);
        SmartDashboard.putNumber("Secondary Velocity", secondaryVelocityDivided);
        SmartDashboard.putNumber("Secondary Setpoint", secondarySetpoint);
    }

    public void shooterPeriodic() {
        mainVelocityDivided = mainEncoder.getVelocity() / 10000;
        secondaryVelocityDivided = secondaryEncoder.getVelocity() / -100;
        mainSetpoint = mainPID.getSetpoint();
        secondarySetpoint = secondaryPID.getSetpoint();
        if (!reversing) {
            if (mainSetpoint == 0.0) {
                mainMotor.set(0);
                mainPID.reset();
            } else {
                mainMotor.set(mainPID.calculate(mainVelocityDivided));
            }
            if (secondarySetpoint == 0.0) {
                secondaryMotor.set(ControlMode.PercentOutput, 0);
                secondaryPID.reset();
            } else {
                secondaryMotor.set(ControlMode.PercentOutput, secondaryPID.calculate(secondaryVelocityDivided));
            }
        }
        reversing = false;
    }

    public void autonomousMode() {
        lowSpeed = new double[] { 95, 0 };
        highFarSpeed = new double[] { 118, 118 };
        highCloseSpeed = new double[] { 121, 121 };
        launchpadSpeed = new double[] { 139, 139 };
    }

    public void teleopMode() {
        lowSpeed = new double[] { 95, 0 };
        highCloseSpeed = new double[] {100, -145 };
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

    
    public void shooterAutoFourth() {
        mainPID.setSetpoint(135);
        secondaryPID.setSetpoint(145);
    }

    public void shooterOff() {
        mainPID.setSetpoint(0.0);
        secondaryPID.setSetpoint(0.0);
    }

    public boolean atSetpoint() {
        return mainPID.atSetpoint() && secondaryPID.atSetpoint();
    }

    public boolean nearSetpoint() {
        mainError = mainPID.getSetpoint()-(mainVelocityDivided);
        secondaryError = secondaryPID.getSetpoint() - (secondaryVelocityDivided);
        return (Math.abs(mainError) <= 0.7) && (Math.abs(secondaryError) <= 0.7);
    }

    public void slowRate() {
        mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0); //Disable
        mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0); //Disable
        mainMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0); //Disable
        secondaryMotor.setStatusFramePeriod(2, 255); // Max Setting
        if(RobotMap.MAX_SLOW_CTRE) {
            for (int status : RobotMap.CTRE_BRUSHED_EXTRA_STATUS_FRAMES) {
                secondaryMotor.setStatusFramePeriod(status, 255);                
            }
        }
    }

}
