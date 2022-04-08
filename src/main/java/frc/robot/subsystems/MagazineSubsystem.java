package frc.robot.subsystems;

import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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

    private int minRange = RobotMap.INDEX_RANGE[0];    
    private int maxRange = RobotMap.INDEX_RANGE[1];
    

    private PWMVictorSPX magazineMotor = new PWMVictorSPX(RobotMap.MAGAZINE_MOTOR_PORT);

    public MagazineSubsystem() {
        Shuffleboard.getTab("Driver Dashboard").addBoolean("Bottom Detector", this::bottomDetector);
        Shuffleboard.getTab("Driver Dashboard").addBoolean("Top Detector", this::topDetector);
    }

    public void lasersharkValues() {
        SmartDashboard.putNumber("Bottom A Range", bottomDetectors[0].getDistanceInches());
        SmartDashboard.putNumber("Bottom B Range", bottomDetectors[1].getDistanceInches());
        SmartDashboard.putNumber("Top A Range", topDetectors[0].getDistanceInches());
        SmartDashboard.putNumber("Top B Range", topDetectors[1].getDistanceInches());
    }

    public void magazineOn() {
        magazineMotor.set(-0.9);
    }
    public void magazineTinyOn() {
        magazineMotor.set(-0.15);
    }
    public void magazineOff() {
        magazineMotor.set(0);
    }

    public boolean bottomDetector() {
        return (bottomDetectors[0].getDistanceInches() > minRange && bottomDetectors[0].getDistanceInches() < maxRange) || (bottomDetectors[1].getDistanceInches() > minRange && bottomDetectors[1].getDistanceInches() < maxRange);
    }
    
    public boolean topDetector() {
        return (topDetectors[0].getDistanceInches() > minRange && topDetectors[0].getDistanceInches() < maxRange) || (topDetectors[1].getDistanceInches() > minRange && topDetectors[1].getDistanceInches() < maxRange);
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
