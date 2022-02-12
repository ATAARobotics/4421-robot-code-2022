package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {

    private DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_PISTONS[0], RobotMap.INTAKE_PISTONS[1]);
    private TalonSRX intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
    
    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void intakeOn() {
        intakePistons.set(Value.kForward);
        intakeMotor.set(ControlMode.PercentOutput, 1);
    }
    public void intakeOff() {
        intakePistons.set(Value.kReverse);
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}
