// package frc.robot.subsystems;

// import com.cuforge.libcu.Lasershark;

// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants;

// public class MagazineSubsystem extends SubsystemBase {
//     private Lasershark[] bottomDetectors = {
//             new Lasershark(Constants.BOTTOM_DETECTOR[0]),
//             new Lasershark(Constants.BOTTOM_DETECTOR[1])
//     };
//     private Lasershark[] topDetectors = {
//             new Lasershark(Constants.TOP_DETECTOR[0]),
//             new Lasershark(Constants.TOP_DETECTOR[1])
//     };

//     private double minRange = Constants.INDEX_RANGE[0];
//     private double maxRange = Constants.INDEX_RANGE[1];

//     private PWMVictorSPX magazineMotor = new PWMVictorSPX(Constants.MAGAZINE_MOTOR_PORT);

//     public MagazineSubsystem() {
//         Shuffleboard.getTab("Driver Dashboard").addBoolean("Bottom Detector", this::bottomDetector);
//         Shuffleboard.getTab("Driver Dashboard").addBoolean("Top Detector", this::topDetector);
//     }

//     @Override
//     public void periodic() {
//         if (Constants.LASERSHARK_DIAGNOSTICS) {
//             lasersharkValues();
//         }
//     }

//     public void lasersharkValues() {
//         SmartDashboard.putNumber("Bottom A Range", bottomDetectors[0].getDistanceInches());
//         SmartDashboard.putNumber("Bottom B Range", bottomDetectors[1].getDistanceInches());
//         SmartDashboard.putNumber("Top A Range", topDetectors[0].getDistanceInches());
//         SmartDashboard.putNumber("Top B Range", topDetectors[1].getDistanceInches());
//     }

//     public void magazineOn() {
//         magazineMotor.set(-1);
//     }

//     public void magazineOff() {
//         magazineMotor.set(0);
//     }

//     public void magazineReverse() {
//         magazineMotor.set(0.4);
//     }

//     public void magazineIndex() {
//         magazineMotor.set(-0.6);
//     }

//     public void magazineIndexShort() {
//         magazineMotor.set(-0.175);
//     }

//     public boolean bottomDetector() {
//         return (bottomDetectors[0].getDistanceInches() > minRange && bottomDetectors[0].getDistanceInches() < maxRange)
//                 || (bottomDetectors[1].getDistanceInches() > minRange
//                         && bottomDetectors[1].getDistanceInches() < maxRange);
//     }

//     public boolean topDetector() {
//         return (topDetectors[0].getDistanceInches() > minRange && topDetectors[0].getDistanceInches() < maxRange)
//                 || (topDetectors[1].getDistanceInches() > minRange && topDetectors[1].getDistanceInches() < maxRange);
//     }

//     public boolean bottomDetectorOnly() {
//         return bottomDetector() && !topDetector();
//     }

//     public boolean topDetectorOnly() {
//         return topDetector() && !bottomDetector();
//     }

//     public boolean bothDetectors() {
//         return bottomDetector() && topDetector();
//     }

//     public boolean bothDetectorsOff() {
//         return !(bottomDetector() && topDetector());
//     }

//     public Trigger getFullMagazineTrigger() {
//         return new FullMagazineTrigger();
//     }

//     public Trigger getEmptyMagazineTrigger() {
//         return new EmptyMagazineTrigger();
//     }
    
//     private class FullMagazineTrigger extends Trigger {
//         @Override
//         public boolean get() {
//             return bothDetectors();
//         }
//     }
    
//     private class EmptyMagazineTrigger extends Trigger {
//         @Override
//         public boolean get() {
//             return bothDetectorsOff();
//         }
//     }
// }
