package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private PWMSparkMax mainMotor = new PWMSparkMax(RobotMap.MAIN_SHOOT_MOTOR_PORT);
    private CANCoder mainEncoder;
    private CANCoder secondaryEncoder;
    private PIDController mainPID = new PIDController(0.015, 0.02, 0.001);

    private PWMVictorSPX secondaryMotor = new PWMVictorSPX(RobotMap.SECONDARY_SHOOT_MOTOR_PORT);
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

    public ShooterSubsystem(String bus) {
        mainMotor.setInverted(true);
        
        CANCoder mainEncoder = new CANCoder(RobotMap.MAIN_SHOOT_ENCODER_ID, bus);    
        CANCoder secondaryEncoder = new CANCoder(RobotMap.SECONDARY_SHOOT_ENCODER_ID, bus);
        this.mainEncoder = mainEncoder;
        this.secondaryEncoder = secondaryEncoder;
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
                secondaryMotor.set(0);
                secondaryPID.reset();
            } else {
                secondaryMotor.set(secondaryPID.calculate(secondaryVelocityDivided));
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
        secondaryMotor.set(-0.2);
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

}
