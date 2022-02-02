package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shooter {
    private boolean intakeOut = false;
    private DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_PISTONS[0], RobotMap.INTAKE_PISTONS[1]);
    
    public Shooter() {

    }

    public void toggleIntake() {
        intakeOut = !intakeOut;
        setIntake(intakeOut);
    }

    public void setIntake(boolean enabled) {
        if (enabled) {
            intakePistons.set(Value.kForward);
        } else {
            intakePistons.set(Value.kReverse);
        }
    }
}
