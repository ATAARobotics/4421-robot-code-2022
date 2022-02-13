package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);
    private Ultrasonic bottomDetector = null;
    private Ultrasonic topDetector = null;

    public MagazineSubsystem() {
        bottomDetector = new Ultrasonic(RobotMap.BOTTOM_DETECTOR[0], RobotMap.BOTTOM_DETECTOR[1]);
        topDetector = new Ultrasonic(RobotMap.TOP_DETECTOR[0], RobotMap.TOP_DETECTOR[1]);

        Ultrasonic.setAutomaticMode(true);
        bottomDetector.setEnabled(true);
        topDetector.setEnabled(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Bottom Detector", bottomDetector.getRangeMM());
        SmartDashboard.putNumber("Top Detector", topDetector.getRangeMM());
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -1);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean bottomDetectorInRange() {
        return bottomDetector.getRangeMM() > RobotMap.INTAKE_RANGE[0] && bottomDetector.getRangeMM() < RobotMap.INTAKE_RANGE[1];
    }
    
    public boolean topDetectorInRange() {
        return topDetector.getRangeMM() > RobotMap.INTAKE_RANGE[0] && topDetector.getRangeMM() < RobotMap.INTAKE_RANGE[1];
    }

    public boolean bottomDetectorOnly() {
        return bottomDetectorInRange() && !topDetectorInRange();
    }
    
    public boolean topDetectorOnly() {
        return topDetectorInRange() && !bottomDetectorInRange();
    }

    public boolean bothDetectors() {
        return bottomDetectorInRange() && topDetectorInRange();
    }
}
