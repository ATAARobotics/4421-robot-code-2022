package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class AutoCommand {

    Trajectory trajectory = null;

    double rotationOffset = -999.0;

    double targetAngle = -999.0;

    //THIS SHOULD ONLY BE USED FOR DATA LOGGING
    List<Translation2d> waypoints = null;
    /**
     * Creates an AutoCommand to move the robot through a set of points, following a cubic spline.
     * 
     * @param rotationOffset The rotation to begin this path with
     * @param waypoints A list of Translation2d objects to pass through in order
     * @param targetAngle The angle (in radians from -Pi to Pi) to turn to during execution of this path
     */
    public AutoCommand(List<Translation2d> waypoints, double targetAngle) {
        this(0, waypoints, targetAngle);
    }

    public AutoCommand(double rotationOffset, List<Translation2d> waypoints, double targetAngle) {
        //Configure the path to not exceed the maximum speed or acceleration specified in Constants
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.MAXIMUM_SPEED, Constants.MAXIMUM_ACCELERATION);

        //Store the rotation offset
        this.rotationOffset = rotationOffset - 0.00001;

        //Store the target angle during execution of this path
        this.targetAngle = targetAngle + 0.00001;

        //Store the waypoints for data logging purposes
        if (Constants.AUTO_PATH_LOGGING_ENABLED) {
            this.waypoints = new ArrayList<Translation2d>(waypoints);
        }

        waypoints = new ArrayList<Translation2d>(waypoints);

        //Get the first and last two points in this path. Some of these may be the same.
        Translation2d firstPoint = waypoints.get(0);
        Translation2d secondPoint = waypoints.get(1);
        Translation2d secondLastPoint = waypoints.get(waypoints.size() - 2);
        Translation2d lastPoint = waypoints.get(waypoints.size() - 1);

        /**
         * Get the angle that the robot should aim for and end with based on the angle to the second and last waypoint.
         * The purpose of this is to prevent the robot from adding a slight bulge to the trajectory, as it thinks that
         * the robot needs to move to be able to turn. Although this would be correct in a differential drive, it is
         * unneccessary with a swerve - so this shaves off a little bit of time by going straight to the waypoints.
         */
        double firstRotation = Math.atan2(secondPoint.getY() - firstPoint.getY(), secondPoint.getX() - firstPoint.getX()); //+ (rotationOffset- 0.00001);
        double lastRotation = Math.atan2(lastPoint.getY() - secondLastPoint.getY(), lastPoint.getX() - secondLastPoint.getX());
        //Remove the first and last waypoints from the list, as we are going to manually specify their rotation
        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putString("Waypoints", waypoints.toString());
        }
        waypoints.remove(0);
        waypoints.remove(waypoints.size() - 1);
        
        System.out.println("first rotation" + firstRotation);
        System.out.println("last rotation" + lastRotation);

        //Create the trajectory based on the waypoints and computed angles
        trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(firstPoint, new Rotation2d(firstRotation)), waypoints, new Pose2d(lastPoint, new Rotation2d(lastRotation)), trajectoryConfig);
    }


    /**
     * Gets the rotation offset at the start of this path if there is one, otherwise returns -999
     * @return
     */
    public double getRotationOffset() {
        return rotationOffset;
    }

    /**
     * Gets the target angle during this command if there is one, otherwise returns -999
     */
    public double getTargetAngle() {
        return targetAngle;
    }


    /**
     * Gets the current state of this path at a timestamp
     * 
     * @param timestamp Time in seconds since this trajectory started
     */
    public State getState(double timestamp) {
        if (trajectory != null) {
            return trajectory.sample(timestamp);
        } else {
            return null;
        }
    }

    /**
     * Gets the last state in this path (used for checking if the path is complete)
     */
    public State getLastState() {
        return trajectory.getStates().get(trajectory.getStates().size() - 1);
    }

    /**
     * For data logging only! Gets a list of all the waypoints in this command
     */
    public List<Translation2d> getWaypoints() {
        return waypoints;
    }
}
