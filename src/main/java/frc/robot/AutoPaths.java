package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPaths {
    /*
     * Path variable declarations, should be formatted as:
     * 
     * private Trajectory pathName;
     */

    private static Trajectory quadrant2EdgeBall5;
    private static Trajectory quadrant2EdgeBall5RED;
    private static Trajectory ball5Ball4;
    private static Trajectory ball5Ball4RED;
    private static Trajectory ball4Ball13;
    private static Trajectory ball4Ball13RED;
    private static Trajectory quadrant1LeftBall2;
    private static Trajectory ball2Launchpad;
    private static Trajectory ball2Ball1;
    private static Trajectory ball1Starve;
    private static Trajectory starveLaunchpad;
    private static Trajectory ball2Quadrant1Line;
    private static Trajectory ball2Quadrant1Wall;
    private static Trajectory leaveTarmac;

    public static void CreateAutoPaths() {

        quadrant2EdgeBall5 = TrajectoryBuilder(
                Math.PI / 2,
                Arrays.asList(
                        new Translation2d(meterConversion(6.4460), meterConversion(7.5447)),
                        new Translation2d(meterConversion(7.75), meterConversion(7.6447))),
                Math.PI / 2 + Math.PI / 12);
        quadrant2EdgeBall5RED = TrajectoryBuilder(
                Math.PI / 2,
                Arrays.asList(
                        new Translation2d(meterConversion(6.4460), meterConversion(7.5447)),
                        new Translation2d(meterConversion(7.75), meterConversion(7.1447))),
                Math.PI / 2 + Math.PI / 12);

        ball5Ball4 = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(7.75), meterConversion(7.6447)),
                        new Translation2d(meterConversion(6.4), meterConversion(6.5)),
                        new Translation2d(meterConversion(6.4), meterConversion(5))),
                13 * Math.PI / 16 + 0.0524);
        ball5Ball4RED = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(7.75), meterConversion(7.1447)),
                        new Translation2d(meterConversion(6.4), meterConversion(6.5)),
                        new Translation2d(meterConversion(6.4), meterConversion(5))),
                13 * Math.PI / 16 + 0.0524);

        ball4Ball13 = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(6.4), meterConversion(5)),
                        new Translation2d(meterConversion(7.5), meterConversion(1.8)),
                        new Translation2d(meterConversion(6), meterConversion(1.2))),
                Math.PI);
        ball4Ball13RED = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(6.4), meterConversion(5)),
                        new Translation2d(meterConversion(7.5), meterConversion(2.0)),
                        new Translation2d(meterConversion(6), meterConversion(1.4))),
                Math.PI);

        quadrant1LeftBall2 = TrajectoryBuilder(
                -2.5724,
                Arrays.asList(
                        new Translation2d(meterConversion(2.9323), meterConversion(6.3812)),
                        new Translation2d(meterConversion(2.0930), meterConversion(5.0693))),
                -2.5724);

        ball2Launchpad = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                        new Translation2d(meterConversion(1.0), meterConversion(4.0))),
                Math.PI / 4);

        ball2Ball1 = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                        new Translation2d(meterConversion(1.0), meterConversion(6.0))),
                -Math.PI / 4);

        ball1Starve = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(1.0), meterConversion(6.0)),
                        new Translation2d(meterConversion(1.0), meterConversion(4.0))),
                -Math.PI);

        starveLaunchpad = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(1.0), meterConversion(4.0)),
                        new Translation2d(meterConversion(0.7), meterConversion(4.5))),
                Math.PI / 4);

        ball2Quadrant1Line = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                        new Translation2d(meterConversion(2.8228), meterConversion(6.2199))),
                -2.5724);

        ball2Quadrant1Wall = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                        new Translation2d(meterConversion(3.8017), meterConversion(7.0141))),
                -2.7751);

        leaveTarmac = TrajectoryBuilder(
                0,
                Arrays.asList(
                        new Translation2d(meterConversion(0), meterConversion(0)),
                        new Translation2d(meterConversion(0), meterConversion(1.25))),
                0);
    }

    public static Trajectory getQuadrant2EdgeBall5() {
        return quadrant2EdgeBall5;
    }

    public static Trajectory getQuadrant2EdgeBall5RED() {
        return quadrant2EdgeBall5RED;
    }

    public static Trajectory getBall5Ball4() {
        return ball5Ball4;
    }

    public static Trajectory getBall5Ball4RED() {
        return ball5Ball4RED;
    }

    public static Trajectory getBall4Ball13() {
        return ball4Ball13;
    }

    public static Trajectory getBall4Ball13RED() {
        return ball4Ball13RED;
    }

    public static Trajectory getQuadrant1LeftBall2() {
        return quadrant1LeftBall2;
    }

    public static Trajectory getBall2Launchpad() {
        return ball2Launchpad;
    }

    public static Trajectory getBall2Ball1() {
        return ball2Ball1;
    }

    public static Trajectory getBall1Starve() {
        return ball1Starve;
    }

    public static Trajectory getStarveLaunchpad() {
        return starveLaunchpad;
    }

    public static Trajectory getBall2Quadrant1Line() {
        return ball2Quadrant1Line;
    }

    public static Trajectory getBall2Quadrant1Wall() {
        return ball2Quadrant1Wall;
    }

    public static Trajectory getLeaveTarmac() {
        return leaveTarmac;
    }

    private static Trajectory TrajectoryBuilder(double rotationOffset, List<Translation2d> waypoints,
            double targetAngle) {
        // Configure the path to not exceed the maximum speed or acceleration specified
        // in RobotMap
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.MAXIMUM_SPEED,
                Constants.MAXIMUM_ACCELERATION);

        // Store the waypoints for data logging purposes
        if (Constants.AUTO_PATH_LOGGING_ENABLED) {
            waypoints = new ArrayList<Translation2d>(waypoints);
        }

        waypoints = new ArrayList<Translation2d>(waypoints);

        // Get the first and last two points in this path. Some of these may be the
        // same.
        Translation2d firstPoint = waypoints.get(0);
        Translation2d secondPoint = waypoints.get(1);
        Translation2d secondLastPoint = waypoints.get(waypoints.size() - 2);
        Translation2d lastPoint = waypoints.get(waypoints.size() - 1);

        /**
         * Get the angle that the robot should aim for and end with based on the angle
         * to the second and last waypoint.
         * The purpose of this is to prevent the robot from adding a slight bulge to the
         * trajectory, as it thinks that
         * the robot needs to move to be able to turn. Although this would be correct in
         * a differential drive, it is
         * unneccessary with a swerve - so this shaves off a little bit of time by going
         * straight to the waypoints.
         */
        double firstRotation = Math.atan2(secondPoint.getY() - firstPoint.getY(),
                secondPoint.getX() - firstPoint.getX());
        double lastRotation = Math.atan2(lastPoint.getY() - secondLastPoint.getY(),
                lastPoint.getX() - secondLastPoint.getX());

        // Remove the first and last waypoints from the list, as we are going to
        // manually specify their rotation
        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putString("Waypoints", waypoints.toString());
        }
        waypoints.remove(0);
        waypoints.remove(waypoints.size() - 1);

        // Create the trajectory based on the waypoints and computed angles
        return TrajectoryGenerator.generateTrajectory(new Pose2d(firstPoint, new Rotation2d(firstRotation)), waypoints,
                new Pose2d(lastPoint, new Rotation2d(lastRotation)), trajectoryConfig);
    }

    // Convert meters to Jacob units
    private static double meterConversion(double meters) {
        return (0.5 * meters) + (0.1811 * Math.signum(meters));
    }

}
