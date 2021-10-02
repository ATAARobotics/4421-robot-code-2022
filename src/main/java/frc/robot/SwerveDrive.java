package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    private Gyro gyro;

    //Whether the swerve should be field-oriented
    boolean fieldOriented = false;

    //An array of all the modules on the swerve drive
    private SwerveModule[] swerveModules;

    //The odometry for the swerve drive
    private SwerveOdometry odometry;

    //The current pose of the robot
    private Pose2d pose;

    //The position that the robot started at
    private Pose2d initialPose;

    //Safety speed override, this *shouldn't* ever be true
    private boolean safetyDisable = false;

    /**
     * Set up the swerve drive
     * 
     * @param gyro The gyro object running on the robot
     * @param initialPose The initial pose that the robot is in
     */
    public SwerveDrive(Gyro gyro, Pose2d initialPose) {
        this.gyro = gyro;
        this.initialPose = initialPose;

        //Initialize four swerve modules using the SwerveModule class
        SwerveModule frontLeftModule = new SwerveModule(new TalonFX(RobotMap.FRONT_LEFT_DRIVE_MOTOR), new TalonFX(RobotMap.FRONT_LEFT_ROTATION_MOTOR), 1.9, true, 0, "Front Left");
        SwerveModule frontRightModule = new SwerveModule(new TalonFX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR), new TalonFX(RobotMap.FRONT_RIGHT_ROTATION_MOTOR), -1.1, false, 1, "Front Right");
        SwerveModule rearLeftModule = new SwerveModule(new TalonFX(RobotMap.REAR_LEFT_DRIVE_MOTOR), new TalonFX(RobotMap.REAR_LEFT_ROTATION_MOTOR), -2.3, true, 2, "Rear Left");
        SwerveModule rearRightModule = new SwerveModule(new TalonFX(RobotMap.REAR_RIGHT_DRIVE_MOTOR), new TalonFX(RobotMap.REAR_RIGHT_ROTATION_MOTOR), 2.1, false, 3, "Rear Right");

        //Put the swerve modules in an array so we can process them easier
        swerveModules = new SwerveModule[]{
            frontLeftModule,
            frontRightModule,
            rearLeftModule,
            rearRightModule
        };

        //Set up odometry
        odometry = new SwerveOdometry(initialPose);

        //Initialize the pose
        pose = initialPose;
    }

    /**
     * This function should be run during every teleop and auto periodic
     */
    public void periodic(SwerveCommand command) {
        if (!safetyDisable) {
            SmartDashboard.putNumber("Gyro Value", gyro.getAngle());

            //Execute functions on each swerve module
            for (SwerveModule module : swerveModules) {
                //Set the drive velocity in meters/second for the module
                module.setDriveVelocity(command.getModuleVelocity(module.getId()));

                //Set module angle target in radians from -Pi to Pi
                module.setTargetAngle(command.getModuleAngle(module.getId()));

                //Run periodic tasks on the module (running motors)
                if (module.periodic()) {
                    //Something has gone horribly wrong if this code is running, there are several checks to prevent it
                    //Abort due to excessive speed
                    safetyDisable = true;
                    break;
                }
            }

            //Update the current pose with the latest command, angle, and a timestamp
            pose = odometry.update(command, gyro.getAngle(), Timer.getFPGATimestamp());
        } else {
            for (SwerveModule module : swerveModules) {
                module.stop();
            }
        }
    }

    /**
     * Sets whether the robot should be field-oriented
     */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    /**
     * Gets whether the robot is field-oriented
     */
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**
     * Gets the robot heading
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    /**
     * Resets the robot heading
     */
    public void resetHeading() {
        gyro.reset();
    }

    /**
     * Gets the current pose of the robot
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Resets the pose to the initial pose
     */
    public void resetPose() {
        pose = initialPose;
        odometry.setPose(pose);
    }
}
