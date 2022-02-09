package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);
    private Lasershark bottomDetector = new Lasershark(0);
    private Lasershark topDetector = new Lasershark(1);
    
    public MagazineSubsystem() {

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -1);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean bottomDetector() {
        return bottomDetector.getDistanceInches() > 0 && bottomDetector.getDistanceInches() < 1;
    }
    
    public boolean topDetector() {
        return topDetector.getDistanceInches() > 0 && topDetector.getDistanceInches() < 1;
    }
}
