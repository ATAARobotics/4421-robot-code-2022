package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */

    private AutoCommand quadrant2EdgeBall5;
    private AutoCommand ball5Quadrant2Edge;
    private AutoCommand quadrant2EdgeBall4;
    private AutoCommand ball4Quadrant2Shoot;
    private AutoCommand quadrant1LeftBall2;
    private AutoCommand ball2Quadrant1Line;
    private AutoCommand ball2Quadrant1Wall;
    private AutoCommand leaveTarmac;

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

        quadrant2EdgeBall5 = new AutoCommand(
            Math.PI / 2,
            Arrays.asList(
                new Translation2d(meterConversion(6.4460), meterConversion(7.5447)),
                new Translation2d(meterConversion(7.65), meterConversion(7.5447))
            ),
            Math.PI / 2
        );

        ball5Quadrant2Edge = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(7.65), meterConversion(7.5447)),
                new Translation2d(meterConversion(6.34), meterConversion(7.5447))
            ),
            1.1 * Math.PI / 2
        );

        quadrant2EdgeBall4 = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6.4460), meterConversion(7.5447)),
                new Translation2d(meterConversion(6.7), meterConversion(5.4252))
            ),
            Math.PI
        );

        ball4Quadrant2Shoot = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6.6570), meterConversion(5.4252)),
                new Translation2d(meterConversion(6.1570), meterConversion(6.5752))
            ),
            3.2 * Math.PI / 4
        );

        quadrant1LeftBall2 = new AutoCommand(
            -2.5724,
            Arrays.asList(
                new Translation2d(meterConversion(2.9323), meterConversion(6.3812)),
                new Translation2d(meterConversion(2.0930), meterConversion(5.0693))
            ),
            -2.5724
        );

        ball2Quadrant1Line = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                new Translation2d(meterConversion(2.8228), meterConversion(6.2199))
            ), 
            -2.5724
        );

        ball2Quadrant1Wall = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(2.0930), meterConversion(5.0693)),
                new Translation2d(meterConversion(3.8017), meterConversion(7.0141))
            ),
            -2.7751
        );

        leaveTarmac = new AutoCommand(
            0,
            Arrays.asList(
                new Translation2d(meterConversion(0), meterConversion(0)),
                new Translation2d(meterConversion(0), meterConversion(1.25))
            ),
            0
        );
    }

    /*  
        Getter functions for paths, formatted as:

        public AutoCommand getPathName() {
            return pathName;
        }
    */

    public AutoCommand getQuadrant2EdgeBall5() {
        return quadrant2EdgeBall5;
    }
    public AutoCommand getBall5Quadrant2Edge() {
        return ball5Quadrant2Edge;
    }
    public AutoCommand getQuadrant2EdgeBall4() {
        return quadrant2EdgeBall4;
    }
    public AutoCommand getBall4Quadrant2Shoot() {
        return ball4Quadrant2Shoot;
    }
    public AutoCommand getQuadrant1LeftBall2() {
        return quadrant1LeftBall2;
    }
    public AutoCommand getBall2Quadrant1Line() {
        return ball2Quadrant1Line;
    }
    public AutoCommand getBall2Quadrant1Wall() {
        return ball2Quadrant1Wall;
    }
    public AutoCommand getLeaveTarmac() {
        return leaveTarmac;
    }

    //Convert meters to Jacob units
    private double meterConversion(double meters) {
        return (0.5*meters) + (0.1811 * Math.signum(meters));
    }
}
