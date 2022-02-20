package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private Lasershark bottomDetector = new Lasershark(0);
    private Lasershark topDetector = new Lasershark(1);
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);

    public MagazineSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Bottom Detector", bottomDetector());
        SmartDashboard.putNumber("Bottom Detector Range", bottomDetector.getDistanceInches());
        SmartDashboard.putBoolean("Top Detector", topDetector());
        SmartDashboard.putNumber("Top Detector Range", topDetector.getDistanceInches());
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -1);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean bottomDetector() {
        return bottomDetector.getDistanceInches() > 0 && bottomDetector.getDistanceInches() < 3;
    }
    
    public boolean topDetector() {
        return topDetector.getDistanceInches() > 0 && topDetector.getDistanceInches() < 3;
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
