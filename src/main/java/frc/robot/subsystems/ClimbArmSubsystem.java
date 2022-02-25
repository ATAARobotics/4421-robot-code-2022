package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbArmSubsystem extends SubsystemBase {
    private DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_ARM[0], RobotMap.CLIMB_ARM[1]);

    public ClimbArmSubsystem() {

    }

    public void armTilt() {
        arm.set(Value.kReverse);
    }
    public void armVertical() {
        arm.set(Value.kForward);
    }
}
