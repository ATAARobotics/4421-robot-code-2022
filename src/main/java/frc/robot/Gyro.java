package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private AHRS navX;
    private double offset;

    public void initializeNavX() {
        try {
            // Initializes the navX object on the roboRIO's MXP port and resets it
            navX = new AHRS(SPI.Port.kMXP);
            navX.reset();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        offset = 0;
        navX.calibrate();
    }
    /**
     * Resets the navX reading to be straight ahead
     */
    public void reset() {
        navX.reset();
    }

    public void setOffset(double rotationOffset) {
        offset = rotationOffset;
    }

    /**
     * Gets the value from the navX, measured in radians from -Pi to Pi
     */
    public double getAngle() {
        double angle = navX.getAngle();

        //Convert to radians
        angle = Math.toRadians(angle) + offset;

        //Offset by Pi to find values in the wrong half of the circle
        angle += Math.PI;

        //Wrap angle at 2*Pi
        angle %= 2.0 * Math.PI;

        //Ensure the value is not negative
        if (angle < 0) {
            angle += 2.0 * Math.PI;
        }

        //Undo the offset
        angle -= Math.PI;

        return angle;
    }
}
