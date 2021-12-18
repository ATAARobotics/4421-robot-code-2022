package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */
    private AutoCommand square;
    private AutoCommand lineX;
    private AutoCommand lineY;

    public AutoPaths() {
        /*  
            Path creation, should be formatted as:

            pathName = new AutoCommand(
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

            The first Translation2d object MUST contain the position of the robot at the time that this command gets executed.
            This should just be done using the previous xEnd and yEnd numbers as the xStart and yStart for any command that follows.

            The endingAngle should be the angle that the robot is at when the path is completed. The robot DOES NOT turn like a differential drive would have to,
            over the course of the path, the robot will turn toward that angle, without regard to the current direction of travel.
        */

        square = new AutoCommand(
            Arrays.asList(
                new Translation2d(0, 0),
                new Translation2d(0, meterConversion(1)),
                new Translation2d(meterConversion(1), meterConversion(1)),
                new Translation2d(meterConversion(1), 0),
                new Translation2d(0, 0)
            ),
            -Math.PI / 2
        );

        lineX = new AutoCommand(
            Arrays.asList(
                new Translation2d(0, 0),
                new Translation2d(meterConversion(5), 0)
            ),
            0
        );

        lineY = new AutoCommand(
            Arrays.asList(
                new Translation2d(0, 0),
                new Translation2d(0, meterConversion(5))
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
    public AutoCommand getSquare() {
        return square;
    }
    public AutoCommand getLineX() {
        return lineX;
    }

    public AutoCommand getLineY() {
        return lineY;
    }
    //Convert meters to Jacob units
    private double meterConversion(double meters) {
        return((0.3917*meters) + 0.1811);
    }
}
