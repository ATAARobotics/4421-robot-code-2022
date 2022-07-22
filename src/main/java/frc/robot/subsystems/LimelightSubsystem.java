package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // read values periodically
    double x;
    double y;
    double area;

    boolean target = false;
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
        camMode = table.getEntry("camMode");
        ledMode = table.getEntry("ledMode");
    }

    @Override
    public void periodic() {
        // read values
        target = tv.getDouble(0) == 1;
        x = (tx.getDouble(0.0) * 2 + x) / 3;
        area = ta.getDouble(0.0);

        // post to smart dashboard
        SmartDashboard.putBoolean("Target Lock", target);
        SmartDashboard.putNumber("Angle to Target", x);
        SmartDashboard.putNumber("Target Area", area);
    }

    private double getAngularDistance() {
        return x * (Math.PI / 180);
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
        if (measurements < 20) {
            if (target) {
                angleTargets[measurements] = getAngularDistance();
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
        return (angleTargets[9] + angleTargets[10]) / 2;
    }
}
