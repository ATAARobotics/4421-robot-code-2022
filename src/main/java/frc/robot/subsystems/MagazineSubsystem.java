package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);
    
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
}
