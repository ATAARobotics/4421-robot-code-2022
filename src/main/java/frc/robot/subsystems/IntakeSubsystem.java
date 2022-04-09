package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_PISTONS[0], RobotMap.INTAKE_PISTONS[1]);
    private PWMVictorSPX intakeMotor = new PWMVictorSPX(RobotMap.INTAKE_MOTOR_PORT);
    
    public IntakeSubsystem() {
        
    }

    public void intakeOn() {
        intakePistons.set(Value.kForward);
        intakeMotor.set(1);
    }
    public void intakeOff() {
        intakePistons.set(Value.kReverse);
        intakeMotor.set(0);
    }
}
