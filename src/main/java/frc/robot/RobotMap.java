package frc.robot;

/**
 * A centralized file that keeps track of constants of the robot, such as motor ports and robot dimensions
 * 
 * This is not the same as the RobotMaps from previous years, the only thing in this class is constants, each hardware class defines its own motors and whatnot
 */
public class RobotMap {
    //Enforces a maximum safe speed of the motors. This may cause steering issues, so this should always be 1 unless debugging
    public static final double MAX_SAFE_SPEED_OVERRIDE = 0.65;

    //Measurements are in meters
    public static final double WHEELBASE = 0.65;
    public static final double TRACK_WIDTH = 0.52;

    //Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 2.0;
    //Used only for auto. Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 0.25;

    //Maximum rotational speed is in radians/second
    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI / 2.0;
    //Maximum rotational acceleration is in radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI / 4.0;

    //Motor ports
    //Drive motors
    public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int REAR_LEFT_DRIVE_MOTOR = 3;
    public static final int REAR_RIGHT_DRIVE_MOTOR = 4;
    //Rotation motors
    public static final int FRONT_LEFT_ROTATION_MOTOR = 5;
    public static final int FRONT_RIGHT_ROTATION_MOTOR = 6;
    public static final int REAR_LEFT_ROTATION_MOTOR = 7;
    public static final int REAR_RIGHT_ROTATION_MOTOR = 8;

    //Encoder ports
    public static final int FRONT_LEFT_ENCODER = 9;
    public static final int FRONT_RIGHT_ENCODER = 10;
    public static final int REAR_LEFT_ENCODER = 11;
    public static final int REAR_RIGHT_ENCODER = 12;

    //Drive encoder ticks per meter
    public static final double FRONT_LEFT_TICKS_PER_METER = 20918;
    public static final double FRONT_RIGHT_TICKS_PER_METER = 15904;
    public static final double REAR_LEFT_TICKS_PER_METER = 17356;
    public static final double REAR_RIGHT_TICKS_PER_METER = 15837;

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
    public static final boolean DETAILED_POSITION_INFORMATION = false;
}
