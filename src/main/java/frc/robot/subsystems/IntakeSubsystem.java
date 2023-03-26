package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.INTAKE_PISTONS[0],
            Constants.INTAKE_PISTONS[1]);
    private PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.INTAKE_MOTOR_PORT);

    public IntakeSubsystem() {

    }

    public void intakeOn() {
        intakePistons.set(Value.kForward);
        intakeMotor.set(0.8);
    }

    public void intakeOff() {
        intakePistons.set(Value.kReverse);
        intakeMotor.set(0);
    }
}
