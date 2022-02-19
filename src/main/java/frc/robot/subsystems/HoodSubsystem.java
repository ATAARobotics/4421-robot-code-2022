package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HoodSubsystem extends SubsystemBase {

    private DoubleSolenoid hoodPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.HOOD_PISTONS[0], RobotMap.HOOD_PISTONS[1]);

    public HoodSubsystem() {

    }

    public void hoodOut() {
        hoodPistons.set(Value.kReverse);
    }

    public void hoodIn() {
        hoodPistons.set(Value.kForward);;
    }
}
