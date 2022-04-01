package frc.robot;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */

    private AutoCommand quadrant2EdgeBall5;
    private AutoCommand quadrant2EdgeBall5RED;
    private AutoCommand ball5Ball4;
    private AutoCommand ball5Ball4RED;
    private AutoCommand ball4Ball13;
    private AutoCommand ball4Ball13RED;
    private AutoCommand ball13Shoot;
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
                new Translation2d(meterConversion(7.75), meterConversion(7.6447))
            ),
            Math.PI / 2 + Math.PI / 12
        );
        quadrant2EdgeBall5RED = new AutoCommand(
            Math.PI / 2,
            Arrays.asList(
                new Translation2d(meterConversion(6.4460), meterConversion(7.5447)),
                new Translation2d(meterConversion(7.75), meterConversion(7.1447))
            ),
            Math.PI / 2 + Math.PI / 12
        );

        ball5Ball4 = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(7.75), meterConversion(7.6447)),
                new Translation2d(meterConversion(6.4), meterConversion(6.5)),
                new Translation2d(meterConversion(6.4), meterConversion(5))
            ),
            13*Math.PI/16+0.0524
        );
        ball5Ball4RED = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(7.75), meterConversion(7.1447)),
                new Translation2d(meterConversion(6.4), meterConversion(6.5)),
                new Translation2d(meterConversion(6.4), meterConversion(5))
            ),
            13*Math.PI/16+0.0524
        );

        ball4Ball13 = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6.4), meterConversion(5)),
                new Translation2d(meterConversion(7.5), meterConversion(1.8)),
                new Translation2d(meterConversion(6), meterConversion(1.8))
            ), 
            7*Math.PI/8
        );
        ball4Ball13RED = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6.4), meterConversion(5)),
                new Translation2d(meterConversion(7.6), meterConversion(1.72)),
                new Translation2d(meterConversion(7), meterConversion(1.7))
            ), 
            7*Math.PI/8
        );

        ball13Shoot = new AutoCommand(
            Arrays.asList(
                new Translation2d(meterConversion(6), meterConversion(1.8)),
                new Translation2d(meterConversion(4.5), meterConversion(2.4))
            ),
            7 * Math.PI / 8
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
    public AutoCommand getQuadrant2EdgeBall5RED() {
        return quadrant2EdgeBall5RED;
    }
    public AutoCommand getBall5Ball4() {
        return ball5Ball4;
    }
    public AutoCommand getBall5Ball4RED() {
        return ball5Ball4RED;
    }
    public AutoCommand getBall4Ball13() {
        return ball4Ball13;
    }
    public AutoCommand getBall4Ball13RED() {
        return ball4Ball13RED;
    }
    public AutoCommand getBall13Shoot() {
        return ball13Shoot;
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
        return (0.5 * meters) + (0.1811 * Math.signum(meters));
    }
}
