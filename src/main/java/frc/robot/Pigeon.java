package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {

    private Pigeon2 pigeon;

    public Pigeon() {
        pigeon = new Pigeon2(Constants.PIGEON_ID, "rio");

        // Mount direction settings - (forward, up) as according to the pigeon's casing
        pigeon.configMountPose(AxisDirection.NegativeZ, AxisDirection.PositiveY);

        // CALIBRATION OF PIGEON (attempt to complete all steps quickly):
        // 1. Drive the robot flush with a flat surface
        // 2. Restart the robot code or redeploy
        // 3. Spin the robot 10 rotations, at full speed, clockwise
        // 4. Drive the robot back to the surface
        // 5. Mark down the heading of the robot
        // 6. Repeat steps 3-4, but spin counterclockwise instead of clockwise
        // 7. Mark down the heading of the robot
        // If the heading from step 7 is not within +-45 degrees, there might be a
        // hardware problem - try mounting the pigeon in a different orientation.
        // Otherwise, to calculate the error:
        // error = ((heading from step 5 - (heading from step 7 / 2)) / 10) * (180 / PI)
        // Repeat the whole process a couple times, tweaking the value, until you end up
        // with a value from step 5 that is very close to half the value from step 7.
        // PUT YOUR ERROR VALUE IN HERE: (was 3.5, mateo)
        pigeon.configYAxisGyroError(0);
        
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

    public double getYawRaw() {
        return pigeon.getYaw();
    }
}
