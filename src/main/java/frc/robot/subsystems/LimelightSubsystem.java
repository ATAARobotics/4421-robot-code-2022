package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Arrays;

public class LimelightSubsystem extends SubsystemBase {

    public enum LimelightState {
        MEASURING,
        FAILED,
        SUCCESS
    }

    private NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry camMode;
    NetworkTableEntry ledMode;
    double[] angleTargets = new double[20];
    int measurements = 0;
    int failedMeasurements = 0;

    public enum CameraMode {
        Vision,
        Driver
    }

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight-forge");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        ty = table.getEntry("ty");
        camMode = table.getEntry("camMode");
        ledMode = table.getEntry("ledMode");
        camMode.setDouble(1);
        ledMode.setDouble(1);
    }

    public void setCameraMode(CameraMode mode) {
        switch (mode) {
            case Vision:
                camMode.setDouble(0);
                ledMode.setDouble(3);
                break;
            case Driver:
                camMode.setDouble(1);
                ledMode.setDouble(1);
                break;
        }
    }

    public void resetTarget() {
        angleTargets = new double[20];
        measurements = 0;
        failedMeasurements = 0;
    }

    public LimelightState measure() {
        SmartDashboard.putString("Limelight State", "Messuring");
        if (measurements < 20) {
            if (tv.getDouble(0) == 1) {
                angleTargets[measurements] = tx.getDouble(0.0) * (Math.PI / 180);
                measurements++;
                if (measurements >= 20) {
                    return LimelightState.SUCCESS;
                } else {
                    return LimelightState.MEASURING;
                }
            } else {
                failedMeasurements++;
                if (failedMeasurements >= 10) {
                    return LimelightState.FAILED;
                } else {
                    return LimelightState.MEASURING;
                }
            }
        } else {
            DriverStation.reportWarning("Attempted to measure limelight target after completing all 20 measurements!",
                    false);
            return LimelightState.SUCCESS;
        }
    }

    /**
     * Get the median of the 20 limelight measurements
     */
    public double getTargetAngle() {
        if (measurements < 20) {
            DriverStation.reportError(
                    "Cannot read limelight target without reaching the required number of measurements - measured "
                            + measurements + "/20 times",
                    false);
            return 0;
        }
        Arrays.sort(angleTargets);
        if(angleTargets.length % 2 == 1){
            return (angleTargets[(int) Math.floor(angleTargets.length/2.0)]);

        }else{
            return (angleTargets[(int) (Math.floor((double) angleTargets.length/2.0) + Math.ceil((double) angleTargets.length/2.0)/2)]);
        }
    }
    
    public double getTargetDistance(){
        double distance = (1.651 / (Math.tan(Constants.LIMELIGHT_ANGLE + (ty.getDouble(0.0) * Math.PI/180)))); 
        System.out.println("the Distance is " + distance + "in meters");
        System.out.println("the ty value is " + ty.getDouble(0.0));
        return distance;
    }
}
