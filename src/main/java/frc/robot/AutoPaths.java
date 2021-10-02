package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class AutoPaths {
    
    //Path variable declarations

    private AutoCommand testPath;

    public AutoPaths() {
        testPath = new AutoCommand(
            Arrays.asList(
                new Translation2d(2.0, 2.0),
                new Translation2d(2.0, 3.0)
            ),
            0.0
        );
    }


    //Getter functions for paths

    public AutoCommand getTestPath() {
        return testPath;
    }
}
