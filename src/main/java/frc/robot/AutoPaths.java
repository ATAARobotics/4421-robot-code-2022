package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */

    private AutoCommand quadrant2WallBall5;
    private AutoCommand ball5Ball4;
    private AutoCommand ball4Quadrant2Wall;
    private AutoCommand quadrant1LeftBall2;
    private AutoCommand ball2Quadrant1Line;

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
                    new Translation2d(xEnd, yEnd),
                ),
                endingAngle
            );

            The rotationOffset term should only be included if this is the first path that will be executed in an auto program,
            AND the robot will not be starting the match pointed straight ahead. If this is the case, this should be equal to
            the heading of the robot, measured in radians, when the match starts.

            The first Translation2d object MUST contain the position of the robot at the time that this command gets executed.
            This should just be done using the previous xEnd and yEnd numbers as the xStart and yStart for any command that follows.

            The endingAngle should be the angle that the robot will attempt to be at when the path is completed. The robot DOES NOT turn like a
            differential drive would have to - over the course of the path, the robot will turn toward that angle, without regard to the current
            direction of travel. This does have the drawback that if the path is too short, the turning may not be complete, and would simply stop.
        */

        quadrant2WallBall5 = new AutoCommand(
            1.9373,
            Arrays.asList(
                new Translation2d(meterConversion(5.3694), meterConversion(7.7480)),
                new Translation2d(meterConversion(7.4558), meterConversion(7.5447))
            ), 
            Math.PI / 2
        );

        ball5Ball4 = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(7.4558), meterConversion(7.5447)),
                new Translation2d(meterConversion(6.6570), meterConversion(5.4252))
            ),
            -0.75 * Math.PI
        );

        ball4Quadrant2Wall = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6.6570), meterConversion(5.4252)),
                new Translation2d(meterConversion(5.3694), meterConversion(7.7480))
            ),
            1.9373
        );

        quadrant1LeftBall2 = new AutoCommand(
            -2.5724,
            Arrays.asList(
                new Translation2d(meterConversion(2.9323), meterConversion(6.3812)),
                new Translation2d(meterConversion(2.2983), meterConversion(5.3902))
            ),
            -2.5724
        );

        ball2Quadrant1Line = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(2.2983), meterConversion(5.3902)),
                new Translation2d(meterConversion(2.6175), meterConversion(5.8990))
            ), 
            -2.5724
        );
    }

    /*  
        Getter functions for paths, formatted as:

        public AutoCommand getPathName() {
            return pathName;
        }
    */

    public AutoCommand getQuadrant2WallBall5() {
        return quadrant2WallBall5;
    }
    public AutoCommand getBall5Ball4() {
        return ball5Ball4;
    }
    public AutoCommand getBall4Quadrant2Wall() {
        return ball4Quadrant2Wall;
    }
    public AutoCommand getQuadrant1LeftBall2() {
        return quadrant1LeftBall2;
    }
    public AutoCommand getBall2Quadrant1Line() {
        return ball2Quadrant1Line;
    }

    //Convert meters to Jacob units
    private double meterConversion(double meters) {
        return (0.3917*meters) + (0.1811 * Math.signum(meters));
    }
}
