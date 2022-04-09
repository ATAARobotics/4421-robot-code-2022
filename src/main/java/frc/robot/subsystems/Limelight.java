package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    private NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry camMode;
    NetworkTableEntry ledMode;

    //read values periodically
    double x;
    double y;
    double area;

    boolean target = false;

    public enum CameraMode {
        Vision,
        Driver
    }

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight-forge");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ta = table.getEntry("ta");
        camMode = table.getEntry("camMode");
        ledMode = table.getEntry("ledMode");
    }

    @Override
    public void periodic() {
        //read values
        target = tv.getDouble(0) == 1;
        x = (tx.getDouble(0.0) * 2 + x) / 3;
        area = ta.getDouble(0.0);

        //post to smart dashboard
        SmartDashboard.putBoolean("Target Lock", target);
        SmartDashboard.putNumber("Angle to Target", x);
        SmartDashboard.putNumber("Target Area", area);
    }

    public boolean hasTarget() {
        return target;
    }

    public double getAngularDistance() {
        return -(x * (Math.PI / 180));
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
}
