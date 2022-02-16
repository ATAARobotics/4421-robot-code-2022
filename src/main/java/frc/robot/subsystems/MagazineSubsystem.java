package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.cuforge.libcu.Lasershark;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class MagazineSubsystem extends SubsystemBase {
    //private Ultrasonic bottomDetector = new Ultrasonic(RobotMap.BOTTOM_DETECTOR[0], RobotMap.BOTTOM_DETECTOR[1]);
    //private Ultrasonic topDetector = new Ultrasonic(RobotMap.TOP_DETECTOR[0], RobotMap.TOP_DETECTOR[1]);
    private Lasershark bottomDetector = new Lasershark(0);
    private Lasershark topDetector = new Lasershark(1);
    private TalonSRX magazineMotor = new TalonSRX(RobotMap.MAGAZINE_MOTOR);

    public MagazineSubsystem() {
        /*Ultrasonic.setAutomaticMode(true);
        bottomDetector.setEnabled(true);
        topDetector.setEnabled(true);*/
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Bottom Detector", bottomDetector.getRangeMM());
        //SmartDashboard.putNumber("Top Detector", topDetector.getRangeMM());
        SmartDashboard.putBoolean("Bottom Detector", bottomDetector());
        SmartDashboard.putNumber("Bottom Detector Range", bottomDetector.getDistanceInches());
        SmartDashboard.putBoolean("Top Detector", topDetector());
        SmartDashboard.putNumber("Top Detector Range", topDetector.getDistanceInches());
    }

    public void magazineOn() {
        magazineMotor.set(ControlMode.PercentOutput, -1);
    }
    public void magazineOff() {
        magazineMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean bottomDetector() {
        return bottomDetector.getDistanceInches() > 0 && bottomDetector.getDistanceInches() < 1;
    }
    
    public boolean topDetector() {
        return topDetector.getDistanceInches() > 0 && topDetector.getDistanceInches() < 1;
    }
    public boolean bottomDetectorOnly() {
        return bottomDetector() && !topDetector();
    }
    
    public boolean topDetectorOnly() {
        return topDetector() && !bottomDetector();
    }
    public boolean bothDetectors() {
        return bottomDetector() && topDetector();
    }
    

    /*public boolean bottomDetectorInRange() {
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
    }*/
}
