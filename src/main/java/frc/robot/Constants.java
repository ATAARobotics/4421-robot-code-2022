package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Preferences;

/**
 * A centralized file that keeps track of constants of the robot, such as device
 * IDs, device ports and robot dimensions
 * 
 * This is not the same as the RobotMaps from previous years, the only thing in
 * this class is constants, each hardware class defines its own motors and
 * whatnot
 */
public class Constants {
    // Disables some safeties and enables logging of warnings we expect and know
    // about during development
    public static final boolean COMP_MODE = false;

    /**
     * Identify which robot this is using Preferences on the rio. This is used to
     * get things like different ticks per meter, offsets, and dimensions.
     * If, for some reason you need to set this, you can put the following commented
     * line into the Robot class' constructor:
     * Preferences.setBoolean("compBot", *VALUE HERE*);
     */
    public static final boolean COMP_BOT = Preferences.getBoolean("compBot", true);

    // Enforces a maximum safe speed of the motors. This may cause steering issues.
    public static final double MAX_SAFE_SPEED_OVERRIDE = COMP_MODE ? 1.0 : 0.8;

    // Measurements are in meters
    public static final double WHEELBASE = COMP_BOT ? 0.65 : 0.65;
    public static final double TRACK_WIDTH = COMP_BOT ? 0.47 : 0.52;

    // Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 2;
    // USED ONLY IN AUTO - Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 2.0;

    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI;
    // Maximum rotational speed is in radians/second Auto
    public static final double MAXIMUM_ROTATIONAL_SPEED_AUTO = Math.PI;
    // USED ONLY IN AUTO - Maximum rotational acceleration is in
    // radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI;

    // Swerve offset
    public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
        2.1138, -0.3758, -2.1506, 0.4740
    }
            : new double[] {
                    0, 0, 0, 0
            };
    /*
     * CAN ID and CAN Bus
     * CAN Bus options supported: "rio", "canivore"
     * ***IF CANIVORE FAILS CHANGE SWERVE_BUS_ACTIVE TO false***
     */

    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final int SECONDARY_SHOOT_ENCODER_ID = 19;

    // CAN FD Device IDs
    public static final int[] DRIVE_MOTORS_ID = { 1, 2, 3, 4 };
    public static final int[] ROTATION_MOTORS_ID = { 5, 6, 7, 8 };
    public static final int[] ROTATION_ENCODERS_ID = { 9, 10, 11, 12 };
    public static final int PIGEON_ID = 20;

    // CAN Legacy Device IDs
    public static final int CLIMB_MOTOR_ID = 13;
    public static final int MAIN_SHOOT_MOTOR_ID = 14;
    public static final int SECONDARY_SHOOT_MOTOR_ID = 15;

    /*
     * CAN Bus (Legacy) NOT CURRENTLY SUPPORTED
     * public static final String SPARK_MOTOR_BUS = "rio";
     */

    // PWM Ports
    public static final int INTAKE_MOTOR_PORT = 0;

    // Sensor Ports
    public static final int[] BOTTOM_DETECTOR = { 0, 1 };
    public static final int[] TOP_DETECTOR = { 2, 3 };
    public static final int[] PASSIVE_HOOK_DETECTORS = { 9, 8 };

    // Solenoid Ports
    public static final int[] CLIMB_ARM = { 6, 7 };
    public static final int[] INTAKE_PISTONS = { 4, 5 };
    public static final int[] HOOD_PISTONS = { 0, 1 };

    // Sensor Config: Intake detector min and max distance to detect a ball
    // (supposedly inches)
    public static final double[] INDEX_RANGE = { 0.5, 3 };

    // Sensor config
    // Intake detector min and max distance to detect a ball (supposedly
    // millimeters)
    public static final double[] INTAKE_RANGE = { 0.0, 75.0 };
    // Periodic ticks to wait before assuming the vision targeting is done
    public static final int TARGETED_TICKS = 10;
    // Angle that the robot can be off by acceptably for the vision targeting to be
    // considered done (radians)
    public static final double VISION_TARGET_TOLERANCE = 0.0523; // This isn't a magic number - it's the central
                                                                 // angle
                                                                 // given from an arc length of 8 inches at a radius
                                                                 // of
                                                                 // 204 inches

    // Drive encoder ticks per meter
    public static final double[] TICKS_PER_METER = COMP_BOT ? new double[] {
            43191.3003, 43777.7504, 43744.6686, 42909.4215
    }
            : new double[] {
                    0, 0, 0, 0
            };

    // DRIVER CONFIG
    // Dead zones of each joystick - Measured from 0 to 1. This should always be at
    // least 0.1.
    public static final double JOY_DEAD_ZONE = 0.3;
    // Whether teleop should start in field oriented mode
    public static final boolean FIELD_ORIENTED = true;
    // The sensitivity value for the joysticks - the values are exponentiated to
    // this value, so higher numbers result in a lower sensitivity, 1 results in
    // normal sensitivity, and decimals increase sensitivity
    public static final double JOYSTICK_SENSITIVITY = 1;
    public static final double TURNING_SENSITIVITY = 3;
    // Time remaining in match to light up the climb indicator
    public static final double CLIMB_TIME = 40;

    //LimeLight Angle
    public static final double LIMELIGHT_ANGLE = 0.5034;
    // LOGGING
    // Set this to true if you want to log diagnostics to SmartDashboard
    public static final boolean REPORTING_DIAGNOSTICS = true;
    // Set this to true if you want to log lasershark values from the magazine to
    // SmartDashboard
    public static final boolean LASERSHARK_DIAGNOSTICS = true;
    // Set this to true if you want to visualize the robot's movement during auto -
    // talk to Jacob if you have no idea what this does
    public static final boolean AUTO_PATH_LOGGING_ENABLED = false;

    public static class VisionConstants {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d CAMERA_TO_ROBOT =
            new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

        public static final AprilTag[] AprilTagPos  = {
                new AprilTag(1, 15.513558, 1.071626, 0.462788, 0.0, 0.0, 0.0, 1.0), 
                new AprilTag(4, 16.178784, 6.749796, 0.695452, 0.0, 0.0, 0.0, 1.0),
                new AprilTag(3, 15.513558, 4.424426, 0.462788, 0.0, 0.0, 0.0, 1.0),
                
                new AprilTag(2, 15.513558, 2.748026, 0.462788, 0.0, 0.0, 0.0, 1.0),
                new AprilTag(5, 0.36195, 6.749796, 0.695452, 1.0, 0.0, 0.0, 0.0),
                
                new AprilTag(6, 1.02743, 4.424426, 0.462788, 1.0, 0.0, 0.0, 0.0),
                new AprilTag(7, 1.02743, 2.748026, 0.462788, 1.0, 0.0, 0.0, 0.0),
                new AprilTag(8, 1.02743, 1.071626, 0.462788, 1.0, 0.0, 0.0, 0.0)
        };
      }

      public static class placementConstants {

                public static enum placements {
                        leftLeftBlue(0),
                        leftMidBlue(1),
                        leftRightBlue(2),

                        midLeftBlue(3),
                        midMidBlue(4),
                        midRightBlue(5),

                        rightLeftBlue(6),
                        rightMidBlue(7),
                        rightRightBlue(8),

                        leftLeftRed(9),
                        leftMidRed(10),
                        leftRightRed(11),

                        midLeftRed(12),
                        midMidRed(13),
                        midRightRed(14),

                        rightLeftRed(15),
                        rightMidRed(16),
                        rightRightRed(17),

                        feederBlue(18),
                        feederRed(19);

                        private final int value;
                        placements(final int newvalue) {
                                value = newvalue;
                        }
                        public int getValue() {
                                return value;
                        }
                };

                public static final Pose2d leftConeBlue  = new Pose2d();
        
        };

    
}
