package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveOdometry {

    //Stores the current position of the robot
    private Pose2d pose;

    //The last time the odometry was updated
    private double lastUpdate = -1.0;

    public SwerveOdometry(Pose2d initialPose) {
        
        this.pose = initialPose;
    }

    /**
     * Updates the current location of the robot
     * @param currentAngle The current angle given by the gyro from -Pi to Pi
     * @param timestamp The current timestamp
     */
    public Pose2d update(double xVelocity, double yVelocity, double currentAngle, double timestamp) {
        //Get the amount of time since the last update
        double period;
        if (lastUpdate >= 0) {
            period = timestamp - lastUpdate;
        } else {
            period = 0.0;
        }

        //Stores the current timestamp as the most recent update
        lastUpdate = timestamp;

        //Get the distance traveled since the last update based on the current velocity
        double distanceX = xVelocity * period;
        double distanceY = yVelocity * period;

        //Updates the position of the robot based on the distance traveled
        pose = new Pose2d(pose.getX() + distanceX, pose.getY() + distanceY, new Rotation2d(currentAngle));

        return pose;
    }

    /**
     * Gets the current pose of the robot as a Pose2d object
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Sets a new pose manually
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
