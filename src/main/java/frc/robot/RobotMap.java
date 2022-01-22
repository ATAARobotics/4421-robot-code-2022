package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A centralized file that keeps track of constants of the robot, such as motor ports and robot dimensions
 * 
 * This is not the same as the RobotMaps from previous years, the only thing in this class is constants, each hardware class defines its own motors and whatnot
 */
public class RobotMap {
    //Robot-specific values
    private static final boolean COMP_BOT = Preferences.getBoolean("compBot", true);

    //Enforces a maximum safe speed of the motors. This may cause steering issues.
    public static final double MAX_SAFE_SPEED_OVERRIDE = 0.75;

    //Measurements are in meters
    public static final double WHEELBASE = COMP_BOT ? 0 : 0.65;
    public static final double TRACK_WIDTH = COMP_BOT ? 0 : 0.52;

    //Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 1.0;
    //Used only for auto. Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 0.5;

    //Maximum rotational speed is in radians/second
    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI;
    //Maximum rotational acceleration is in radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI;

    //Swerve offset
    public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
        0, 0, 0, 0
    } : new double[] {
        2.218, 2.598, -1.810, 1.152
    };

    //Motor ports
    //Drive motors
    public static final int[] DRIVE_MOTORS = {
        1, 2, 3, 4
    };
    //Rotation motors
    public static final int[] ROTATION_MOTORS = {
        5, 6, 7, 8
    };

    //Encoder ports
    public static final int[] ROTATION_ENCODERS = {
        9, 10, 11, 12
    };

    //Drive encoder ticks per meter
    public static final double[] TICKS_PER_METER = COMP_BOT ? new double[] {
        0, 0, 0, 0
    } : new double[] {
        42651.7831, 43289.4436, 43141.0841, 42732.3823
    };

    //DRIVER CONFIG
    //Dead zones of each joystick - Measured from 0 to 1. This should always be at least 0.1.
    public static final double JOY_DEAD_ZONE = 0.3;
    //Whether teleop should start in field oriented mode
    public static final boolean FIELD_ORIENTED = true;

    //LOGGING
    //Set this to true if you want to visualize the robot's movement during auto
    public static final boolean AUTO_PATH_LOGGING_ENABLED = false;
    //Set this to true if you want detailed Shuffleboard info on each module
    public static final boolean DETAILED_MODULE_INFORMATION = true;
    //Set this to true if you want detailed Shuffleboard info on each module's encoder
    public static final boolean DETAILED_ENCODER_INFORMATION = true;
    //Set this to true if you want detailed Shuffleboard info on the joysticks
    public static final boolean DETAILED_JOYSTICK_INFORMATION = false;
    //Set this to true if you want detailed Shuffleboard info on the position of the robot.
    //From the perspective of the driver, -X is left, +X is right, -Y is backward, and +Y is forward
    public static final boolean DETAILED_POSITION_INFORMATION = true;
}
