package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOdometry {

    //Stores the current position of the robot
    private Pose2d pose;
    private Pigeon pigeon;
    //The last time the odometry was updated
    private double lastUpdate = 0.0;

    private boolean isInitialized = false;

    public SwerveOdometry(Pose2d initialPose, Pigeon pigeon) {
        this.pose = initialPose;
        this.pigeon = pigeon;
    }

    /**
     * Updates the current location of the robot
     * @param currentAngle The current angle given by the gyro from -Pi to Pi
     * @param timestamp The current timestamp
     */
    public Pose2d update(double xVelocity, double yVelocity, double currentAngle, double timestamp) {
        
        if (!isInitialized) {
            return new Pose2d(0, 0, new Rotation2d(0));
        } 
        //Get the amount of time since the last update
        double period = timestamp - lastUpdate;

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

    // take the average of the 2 poses
    public void addAprilTag(Pose2d pose) {

        if (!isInitialized) {
            setPose(pose);
            pigeon.setYaw(pose.getRotation().getDegrees());
            isInitialized = true;
        }

        double x, y, rot;
        x = (this.pose.getX() + pose.getX()) / 2.0;
        y = (this.pose.getY() + pose.getY()) / 2.0;
        rot = (this.pose.getRotation().getRadians() + pose.getRotation().getRadians()) / 2.0;
        this.pose = new Pose2d(x, y, new Rotation2d(rot));
        System.out.println("MERGE APRILTAG LOCATION");
        SmartDashboard.putNumber("Robot Pose X: ", this.pose.getX());
        SmartDashboard.putNumber("Robot Pose Y: ", this.pose.getY());
        SmartDashboard.putNumber("Robot Pose Rot: ", this.pose.getRotation().getDegrees());
    }
}
