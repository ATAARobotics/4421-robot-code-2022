package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);
    //private Lasershark bottomDetector = new Lasershark(0);
    //private Lasershark topDetector = new Lasershark(1);

    public MagazineSubsystem() {

    }

    @Override
    public void periodic() {
      //SmartDashboard.putNumber("Bottom Detector", bottomDetector.getDistanceInches());
      //SmartDashboard.putNumber("Top Detector", topDetector.getDistanceInches());
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -1);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    /*public boolean bottomDetector() {
        return bottomDetector.getDistanceInches() > 0 && bottomDetector.getDistanceInches() < 1;
    }
    
    public boolean topDetector() {
        return topDetector.getDistanceInches() > 0 && topDetector.getDistanceInches() < 1;
    }

    public boolean bottomDetectorOnly() {
        return bottomDetector() && !topDetector();
    }
    
    public boolean topDetectorOnly() {
        return topDetector() && !bottomDetector();
    }

    public boolean bothDetectors() {
        return bottomDetector() && topDetector();

    }*/
}
