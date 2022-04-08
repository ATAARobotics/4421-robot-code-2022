package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbMotorSubsystem extends SubsystemBase {
    private CANSparkMax climbMotor = new CANSparkMax(RobotMap.CLIMB_MOTOR_ID, MotorType.kBrushless);
    private DigitalInput elevatorDownDetector = new DigitalInput(6);
    private double climbMotorSpeed = 0.85;


    public ClimbMotorSubsystem() {
        climbMotor.setInverted(true);
    }

    public void diagnostic() {
        SmartDashboard.putBoolean("Climber Down", climberMin());
        SmartDashboard.putNumber("speed", climbMotorSpeed);
    }

    public void climberSlowSpeed() {
        setSpeed(1);
    }
    public void climberNormalSpeed() {
        setSpeed(2);
    }
    public void climberMaxSpeed() {
        setSpeed(3);
    }

    public void setSpeed(int speedLevel) {
        switch (speedLevel) {
            case 1:
                climbMotorSpeed = 0.3;
                break;
            
            case 2:
                climbMotorSpeed = 0.85;
                break;

            case 3:
                climbMotorSpeed = 1.0;
                break;
        
            default:
                DriverStation.reportError(speedLevel + " is not a valid speed level for the climber!", false);
                break;
        }
    }

    public void climberUp() {
        climbMotor.set(climbMotorSpeed);
    }

    public void climberDown() {
        if (!climberMin()) {
            climbMotor.set(-climbMotorSpeed);
        } else {
            climbMotor.set(0.0);
        }
    }

    public void climberStop() {
        climbMotor.set(0.0);
    }

    public boolean climberMin() {
        return !elevatorDownDetector.get();
    }

    public void slowRate() {
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0); //Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0); //Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0); //Disable
    }
}
