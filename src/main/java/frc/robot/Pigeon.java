package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

public class Pigeon {

    private Pigeon2 pigeon;

    public Pigeon() {
        pigeon = new Pigeon2(RobotMap.PIGEON_ID, "canivore");

        pigeon.configMountPose(AxisDirection.NegativeZ, AxisDirection.PositiveY);
        pigeon.configYAxisGyroError(3.5);

        pigeon.setYaw(0);
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    public double getYaw() {
        double yaw = -pigeon.getYaw();

        yaw *= Math.PI / 180.0;

        yaw %= Math.PI * 2;

        yaw += Math.PI * 3;

        yaw %= Math.PI * 2;

        yaw -= Math.PI;

        return yaw;
    }
}
