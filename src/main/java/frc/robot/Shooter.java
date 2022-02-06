package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shooter {
    private boolean intakeOut = false;
    private DoubleSolenoid intakePistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_PISTONS[0], RobotMap.INTAKE_PISTONS[1]);
    private CANSparkMax shootMotor = new CANSparkMax(RobotMap.SHOOT_MOTOR, MotorType.kBrushless);
    private TalonSRX intakeMotor = new TalonSRX(RobotMap.INTAKE_MOTOR);
    private boolean shootEnabled = false;
    private int shootCase = -1;
    
    public Shooter() {

    }

    public void toggleIntake() {
        intakeOut = !intakeOut;
        setIntake(intakeOut);
    }

    public void setIntake(boolean enabled) {
        if (enabled) {
            intakePistons.set(Value.kForward);
            intakeMotor.set(ControlMode.PercentOutput, 0.5);
        } else {
            intakePistons.set(Value.kReverse);
        }
    }

    public void toggleShooter(int shootCase) {
       this.shootCase = shootCase;
       shootEnabled = !shootEnabled;
    }

    public void shooterPeriodic() {
        if (shootEnabled) {
            switch (shootCase) {
                case 0:
                    shootMotor.set(0.5);
                    break;
                
                case 1:
                    shootMotor.set(0);
                    break;
                
                default:
                    shootMotor.set(0);
                    break;
            }
        }
        else {
            shootMotor.set(0);
        }
    }
}
