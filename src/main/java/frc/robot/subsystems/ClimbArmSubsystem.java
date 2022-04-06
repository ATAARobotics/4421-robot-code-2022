package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbArmSubsystem extends SubsystemBase {
    private DoubleSolenoid arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_ARM[0], RobotMap.CLIMB_ARM[1]);
    private DigitalInput[] passiveHookDetectors = {
        new DigitalInput(RobotMap.PASSIVE_HOOK_DETECTORS[0]),
        new DigitalInput(RobotMap.PASSIVE_HOOK_DETECTORS[1])
    };
    private DoubleSolenoid clamps = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.CLIMB_CLAMPS[0], RobotMap.CLIMB_CLAMPS[1]);

    private boolean armIsTilted = false;
    private boolean forceTilt = true;
    private boolean clamped = false;
    private boolean forceClamp = true;

    public ClimbArmSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Left Hook", passiveHookDetectors[0].get());
        SmartDashboard.putBoolean("Right Hook", passiveHookDetectors[1].get());
        SmartDashboard.putBoolean("Engaged", armEngaged());
    }

    public void clamp() {
        if (!clamped || forceClamp) {
            clamps.set(Value.kForward);
        }
        clamped = true;
        forceClamp = false;
    }
    public void releaseClamps() {
        if (clamped || forceClamp) {
            clamps.set(Value.kReverse);
        }
        clamped = false;
        forceClamp = false;
    }

    public void armTilt() {
        if (!armIsTilted || forceTilt) {
            arm.set(Value.kReverse);
        }
        armIsTilted = true;
        forceTilt = false;
    }
    public void armVertical() {
        if (armIsTilted || forceTilt) {
            arm.set(Value.kForward);
        }
        armIsTilted = false;
        forceTilt = false;
    }

    public boolean armEngaged() {
        return passiveHookDetectors[0].get() && passiveHookDetectors[1].get();
    }
}
