package frc.robot;


import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */
    private AutoCommand straight;
    private AutoCommand straight1;
    private AutoCommand quadrant1LeftBall2;
    private AutoCommand ball2Launchpad;

    public AutoPaths() {
        /*  
            Path creation, should be formatted as:

            pathName = new AutoCommand(
                rotationOffset, **DEFAULTS TO ZERO, THIS IS OPTIONAL**
                Arrays.asList(
                    new Translation2d(xStart, yStart),
                    new Translation2d(xWaypoint1, yWaypoint1),
                    new Translation2d(xWaypoint2, yWaypoint2),
                    new Translation2d(xWaypoint3, yWaypoint3),
                    ...
                    new Translation2d(x, y),
                ),
                ingAngle
            );

            The rotationOffset term should only be included if this is the first path that will be executed in an auto program,
            AND the robot will not be starting the match pointed straight ahead. If this is the case, this should be equal to
            the heading of the robot, measured in radians, when the match starts.

            The first Translation2d object MUST contain the position of the robot at the time that this command gets executed.
            This should just be done using the previous x and y numbers as the xStart and yStart for any command that follows.

            The ingAngle should be the angle that the robot will attempt to be at when the path is completed. The robot DOES NOT turn like a
            differential drive would have to - over the course of the path, the robot will turn toward that angle, without regard to the current
            direction of travel. This does have the drawback that if the path is too short, the turning may not be complete, and would simply stop.
        */

        straight = new AutoCommand(
            Math.PI,
            Arrays.asList(
                new Translation2d(5, 5),
                new Translation2d(5, 3)
            ),
            Math.PI/2
        );
        straight1 = new AutoCommand(
            Math.PI/2,
            Arrays.asList(
                new Translation2d(5, 3),
                new Translation2d(3, 3)
            ),
            Math.PI
        );
        quadrant1LeftBall2 = new AutoCommand(
            -2.5724,
            Arrays.asList(
                new Translation2d(meterConversion(2.9323), meterConversion(5.0693)),
                new Translation2d(meterConversion(2.0930), meterConversion(6.3812))
            ),
            -2.5724
        );
        ball2Launchpad = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                new Translation2d(meterConversion(1), meterConversion(4.0))
            ),
            Math.PI/4
        );
    }

    /*  
        Getter functions for paths, formatted as:

        public AutoCommand getPathName() {
            return pathName;
        }
    */

    public AutoCommand getStraight() {
        return straight;
    }

    public AutoCommand getStraight1() {
        return straight1;
    }
    public AutoCommand GetQuadrant1LeftBall2(){
        return quadrant1LeftBall2;
    }

    public AutoCommand GetBall2Launchpad(){
        return ball2Launchpad;
    }

    //Convert meters to Jacob units
    private double meterConversion(double meters) {
        return (0.5 * meters) + (0.1811 * Math.signum(meters));
    }
}
