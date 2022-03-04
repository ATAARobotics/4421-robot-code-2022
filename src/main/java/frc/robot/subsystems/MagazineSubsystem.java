package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private Lasershark[] bottomDetectors = {
        new Lasershark(RobotMap.BOTTOM_DETECTOR[0]),
        new Lasershark(RobotMap.BOTTOM_DETECTOR[1])
    };
    private Lasershark[] topDetectors = {
        new Lasershark(RobotMap.TOP_DETECTOR[0]),
        new Lasershark(RobotMap.TOP_DETECTOR[1])
    };
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);

    public MagazineSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bottom Detector", bottomDetector());
        SmartDashboard.putNumber("Bottom A Range", bottomDetectors[0].getDistanceInches());
        SmartDashboard.putNumber("Bottom B Range", bottomDetectors[1].getDistanceInches());
        SmartDashboard.putBoolean("Top Detector", topDetector());
        SmartDashboard.putNumber("Top A Range", topDetectors[0].getDistanceInches());
        SmartDashboard.putNumber("Top B Range", topDetectors[1].getDistanceInches());
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -0.9);
    }
    public void magazineTinyOn() {
        magazineMotor.set(ControlMode.PercentOutput, -0.1);
    }
    public void magazineReverse() {
        magazineMotor.set(ControlMode.PercentOutput, 0.4);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean bottomDetector() {
        return (bottomDetectors[0].getDistanceInches() > 0 && bottomDetectors[0].getDistanceInches() < 4) || (bottomDetectors[1].getDistanceInches() > 0 && bottomDetectors[1].getDistanceInches() < 4);
    }
    
    public boolean topDetector() {
        return (topDetectors[0].getDistanceInches() > 0 && topDetectors[0].getDistanceInches() < 4) || (topDetectors[1].getDistanceInches() > 0 && topDetectors[1].getDistanceInches() < 4);
    }
    
    public boolean bottomDetectorOnly() {
        return bottomDetector() && !topDetector();
    }
    public boolean topDetectorOnly() {
        return topDetector() && !bottomDetector();
    }
    public boolean bothDetectors() {
        return bottomDetector() && topDetector();
    }

    public Trigger getFullMagazineTrigger() {
        return new FullMagazineTrigger();
    }
    
    private class FullMagazineTrigger extends Trigger {
        @Override
        public boolean get() {
            return bothDetectors();
        }
    }

}
