package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

//TODO Whoever sees this message, this probably doesnt work - fix this before you do whatever you were planning on doing

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
