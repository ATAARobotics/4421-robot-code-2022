package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A centralized file that keeps track of constants of the robot, such as motor ports and robot dimensions
 * 
 * This is not the same as the RobotMaps from previous years, the only thing in this class is constants, each hardware class defines its own motors and whatnot
 */
public class RobotMap {
    /**
     * Identify which robot this is using Preferences on the rio. This is used to get things like different ticks per meter, offsets, and dimensions.
     * If, for some reason you need to set this, you can put the following commented line into the Robot class' constructor:
     * Preferences.setBoolean("compBot", *VALUE HERE*);
     */
    private static final boolean COMP_BOT = Preferences.getBoolean("compBot", true);

    //Enforces a maximum safe speed of the motors. This may cause steering issues.
    public static final double MAX_SAFE_SPEED_OVERRIDE = 0.75;

    //Measurements are in meters
    public static final double WHEELBASE = COMP_BOT ? 0.65 : 0.65;
    public static final double TRACK_WIDTH = COMP_BOT ? 0.47 : 0.52;

    //Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 1.0;
    //Used only for auto. Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 2.0;

    //Maximum rotational speed is in radians/second
    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI;
    //Maximum rotational acceleration is in radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI;

    //Swerve offset
    public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
        2.1138, -0.3758, -2.1506, 0.4740
    } : new double[] {
        0, 0, 0, 0
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
    //Climber motor
    public static final int CLIMB_MOTOR = 13;
    //Intake motor
    public static final int INTAKE_MOTOR = 15;
    //Magazine motor
    public static final int MAGAZINE_MOTOR = 16;
    //Shooter motors
    public static final int MAIN_SHOOT_MOTOR = 14;
    public static final int SECONDARY_SHOOT_MOTOR = 18;

    //Solenoid ports
    public static final int[] CLIMB_ARM = { 6, 7 };
    public static final int[] INTAKE_PISTONS = { 4, 5 };
    public static final int[] HOOD_PISTONS = { 0, 1 };

    //Encoder ports
    public static final int[] ROTATION_ENCODERS = {
        9, 10, 11, 12
    };
    public static final int MAIN_SHOOT_ENCODER = 17;
    public static final int SECONDARY_SHOOT_ENCODER = 19;

    //Sensor ports
    //Bottom intake detector
    public static final int[] BOTTOM_DETECTOR = { 0, 1 };
    //Top intake detector
    public static final int[] TOP_DETECTOR = { 2, 3 };

    //Sensor config
    //Intake detector min and max distance to detect a ball (supposedly millimeters)
    public static final double[] INTAKE_RANGE = { 0.0, 75.0 };
    //Periodic ticks to wait before assuming the vision targeting is done
    public static final int TARGETED_TICKS = 10;
    //Angle that the robot can be off by acceptably for the vision targeting to be considered done (radians)
    public static final double VISION_TARGET_TOLERANCE = 0.0392; //This isn't a magic number - it's the central angle given from an arc length of 8 inches at a radius of 204 inches

    //Drive encoder ticks per meter
    public static final double[] TICKS_PER_METER = COMP_BOT ? new double[] {
        43191.3003, 43777.7504, 43744.6686, 42909.4215
    } : new double[] {
        0, 0, 0, 0
    };

    //DRIVER CONFIG
    //Dead zones of each joystick - Measured from 0 to 1. This should always be at least 0.1.
    public static final double JOY_DEAD_ZONE = 0.3;
    //Whether teleop should start in field oriented mode
    public static final boolean FIELD_ORIENTED = true;
    //The sensitivity value for the joysticks - the values are exponentiated to this value, so higher numbers result in a lower sensitivity, 1 results in normal sensitivity, and decimals increase sensitivity
    public static final double JOYSTICK_SENSITIVITY = 2;
    public static final double TURNING_SENSITIVITY = 6;

    //LOGGING
    //Set this to true if you want system info of the robot (temperature, battery, etc.)
    public static final boolean ROBOT_INFO = false;
    //Set this to true if you want to visualize the robot's movement during auto
    public static final boolean AUTO_PATH_LOGGING_ENABLED = false;
    //Set this to true if you want detailed Shuffleboard info on each module
    public static final boolean DETAILED_MODULE_INFORMATION = false;
    //Set this to true if you want detailed Shuffleboard info on each module's encoder (ticks per inch)
    public static final boolean DETAILED_ENCODER_INFORMATION = false;
    //Set this to true if you want detailed Shuffleboard info on the joysticks
    public static final boolean DETAILED_JOYSTICK_INFORMATION = false;
    //Set this to true if you want detailed Shuffleboard info on the position of the robot.
    //From the perspective of the driver, -X is left, +X is right, -Y is backward, and +Y is forward
    public static final boolean DETAILED_POSITION_INFORMATION = false;
}
