package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {

    private DoubleSolenoid hoodPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HOOD_PISTONS[0],
            Constants.HOOD_PISTONS[1]);

    private boolean hoodIsOut = false;
    private boolean force = true;

    public HoodSubsystem() {

    }

    // Hood is SUCKED BACK
    public void hoodIn() {
        if (!hoodIsOut || force) {
            hoodPistons.set(Value.kReverse);
        }
        hoodIsOut = true;
        force = false;
    }

    // Hood is TILTED FORWARD
    public void hoodOut() {
        if (hoodIsOut || force) {
            hoodPistons.set(Value.kForward);
        }
        hoodIsOut = false;
        force = false;
    }
}