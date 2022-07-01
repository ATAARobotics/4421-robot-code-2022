package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbArmSubsystem extends SubsystemBase {
    private DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMB_ARM[0],
            Constants.CLIMB_ARM[1]);
    private DigitalInput[] passiveHookDetectors = {
            new DigitalInput(Constants.PASSIVE_HOOK_DETECTORS[0]),
            new DigitalInput(Constants.PASSIVE_HOOK_DETECTORS[1])
    };

    private boolean armIsTilted = false;

    public ClimbArmSubsystem() {

    }

    public void armTilt() {
        if (!armIsTilted) {
            arm.set(Value.kReverse);
        }
        armIsTilted = true;
    }

    public void armVertical() {
        if (armIsTilted) {
            arm.set(Value.kForward);
        }
        armIsTilted = false;
    }

    public boolean engaged() {
        return passiveHookDetectors[0].get() && passiveHookDetectors[1].get();
    }

    public boolean fullyDisengaged() {
        return !passiveHookDetectors[0].get() && !passiveHookDetectors[1].get();
    }
}
