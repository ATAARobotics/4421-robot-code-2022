package frc.robot;

public class AutoPaths {
    
    /*  
        Path variable declarations, should be formatted as:

        private AutoCommand pathName;
    */

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
    }

    /*  
        Getter functions for paths, formatted as:

        public AutoCommand getPathName() {
            return pathName;
        }
    */
}
