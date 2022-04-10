package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A centralized file that keeps track of constants of the robot, such as device IDs, device ports and robot dimensions
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
    public static final double MAX_SAFE_SPEED_OVERRIDE = 0.8;

    //Measurements are in meters
    public static final double WHEELBASE = COMP_BOT ? 0.65 : 0.65;
    public static final double TRACK_WIDTH = COMP_BOT ? 0.47 : 0.52;

    //Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 1.25;
    //USED ONLY IN AUTO - Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 2.0;

    //Maximum rotational speed is in radians/second
    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI;
    //USED ONLY IN AUTO - Maximum rotational acceleration is in radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI;

    //Swerve offset
    public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
        2.1138, -0.3758, -2.1506, 0.4740
    } : new double[] {
        0, 0, 0, 0
    };

    /* 
     * CAN ID and CAN Bus
     * CAN Bus options supported: "rio", "canivore"
     * ***IF CANIVORE FAILS CHANGE SWERVE_BUS_ACTIVE TO false***
     */

    //CAN FD Device IDs 
    public static final int[] DRIVE_MOTORS_ID = {1, 2, 3, 4};
    public static final int[] ROTATION_MOTORS_ID = {5, 6, 7, 8};    
    public static final int[] ROTATION_ENCODERS_ID = {9, 10, 11, 12};

    //CAN Legacy Device IDs
    public static final int CLIMB_MOTOR_ID = 13;
    public static final int MAIN_SHOOT_MOTOR_ID = 14;
    public static final int SECONDARY_SHOOT_MOTOR_ID = 15;

    /* CAN Bus (Legacy) NOT CURRENTLY SUPPORTED
    public static final String SPARK_MOTOR_BUS = "rio";
    */
    
    //PWM Ports
    public static final int INTAKE_MOTOR_PORT = 0;
    public static final int MAGAZINE_MOTOR_PORT = 1;


    //Sensor Ports
    public static final int[] BOTTOM_DETECTOR = { 0, 1 };
    public static final int[] TOP_DETECTOR = { 2, 3 };

    //Solenoid Ports
    public static final int[] CLIMB_ARM = { 6, 7 };
    public static final int[] INTAKE_PISTONS = { 4, 5 };
    public static final int[] HOOD_PISTONS = { 0, 1 };

    //Sensor Config: Intake detector min and max distance to detect a ball (supposedly inches)
    public static final int[] INDEX_RANGE = { 1, 3 };
  
    //Sensor config
    //Intake detector min and max distance to detect a ball (supposedly millimeters)
    public static final double[] INTAKE_RANGE = { 0.0, 75.0 };
    //Periodic ticks to wait before assuming the vision targeting is done
    public static final int TARGETED_TICKS = 10;
    //Angle that the robot can be off by acceptably for the vision targeting to be considered done (radians)
    public static final double VISION_TARGET_TOLERANCE = 0.0523; //This isn't a magic number - it's the central angle given from an arc length of 8 inches at a radius of 204 inches

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
    public static final double TURNING_SENSITIVITY = 5;
    //Time remaining in match to light up the climb indicator
    public static final double CLIMB_TIME = 40;

    //LOGGING
    //Set this to true if you want to log diagnostics to SmartDashboard
    public static final boolean REPORTING_DIAGNOSTICS = false;
    //Set this to true if you want to log lasershark values from the magazine to SmartDashboard
    public static final boolean LASERSHARK_DIAGNOSTICS = true;
    //Set this to true if you want to visualize the robot's movement during auto - talk to Jacob if you have no idea what this does
    public static final boolean AUTO_PATH_LOGGING_ENABLED = false;
}
