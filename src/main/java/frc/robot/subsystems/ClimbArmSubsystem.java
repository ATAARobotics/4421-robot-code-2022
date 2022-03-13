package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbArmSubsystem extends SubsystemBase {
    private DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_ARM[0], RobotMap.CLIMB_ARM[1]);
    private DigitalInput[] passiveHookDetectors = {
        new DigitalInput(RobotMap.PASSIVE_HOOK_DETECTORS[0]),
        new DigitalInput(RobotMap.PASSIVE_HOOK_DETECTORS[1])
    };

    private boolean armIsTilted = false;
    private boolean force = true;

    public ClimbArmSubsystem() {

    }

    public void armTilt() {
        if (!armIsTilted || force) {
            arm.set(Value.kReverse);
        }
        armIsTilted = true;
        force = false;
    }
    public void armVertical() {
        if (armIsTilted || force) {
            arm.set(Value.kForward);
        }
        armIsTilted = false;
        force = false;
    }

    public boolean armEngaged() {
        return passiveHookDetectors[0].get() && passiveHookDetectors[1].get();
    }
}
