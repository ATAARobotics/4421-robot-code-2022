package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {

    private Pigeon2 pigeon;

    public Pigeon() {
        pigeon = new Pigeon2(RobotMap.PIGEON_ID);

        pigeon.setYaw(0);
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }

    public double getYaw() {
        return pigeon.getYaw();
    }
}
