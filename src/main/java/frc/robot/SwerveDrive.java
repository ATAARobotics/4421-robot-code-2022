package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private double driveMotorHighestTemp = 0;
    private double rotationMotorHighestTemp = 0;

    //Safety speed override, this *shouldn't* ever be true
    private boolean safetyDisable = false;

    /**
     * Set up the swerve drive
     * 
     * @param gyro The gyro object running on the robot
     * @param initialPose The initial pose that the robot is in
     */
    public SwerveDrive(Gyro gyro, Translation2d initialPosition) {
        this.gyro = gyro;
        this.initialPose = new Pose2d(initialPosition, new Rotation2d(0.0));

        //Initialize four swerve modules using the SwerveModule class
        SwerveModule frontLeftModule = new SwerveModule(new TalonFX(RobotMap.DRIVE_MOTORS[0]), new TalonFX(RobotMap.ROTATION_MOTORS[0]), new CANCoder(RobotMap.ROTATION_ENCODERS[0]), RobotMap.ANGLE_OFFSET[0], true, RobotMap.TICKS_PER_METER[0], 0, "Front Left");
        SwerveModule frontRightModule = new SwerveModule(new TalonFX(RobotMap.DRIVE_MOTORS[1]), new TalonFX(RobotMap.ROTATION_MOTORS[1]), new CANCoder(RobotMap.ROTATION_ENCODERS[1]), RobotMap.ANGLE_OFFSET[1], false, RobotMap.TICKS_PER_METER[1], 1, "Front Right");
        SwerveModule rearLeftModule = new SwerveModule(new TalonFX(RobotMap.DRIVE_MOTORS[2]), new TalonFX(RobotMap.ROTATION_MOTORS[2]), new CANCoder(RobotMap.ROTATION_ENCODERS[2]), RobotMap.ANGLE_OFFSET[2], true, RobotMap.TICKS_PER_METER[2], 2, "Rear Left");
        SwerveModule rearRightModule = new SwerveModule(new TalonFX(RobotMap.DRIVE_MOTORS[3]), new TalonFX(RobotMap.ROTATION_MOTORS[3]), new CANCoder(RobotMap.ROTATION_ENCODERS[3]), RobotMap.ANGLE_OFFSET[3], false, RobotMap.TICKS_PER_METER[3], 3, "Rear Right");

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

            if (RobotMap.DETAILED_POSITION_INFORMATION) {
                SmartDashboard.putNumber("Distance X", odometry.getPose().getX());
                SmartDashboard.putNumber("Distance Y", odometry.getPose().getY());
            }
        } else {
            for (SwerveModule module : swerveModules) {
                module.stop();
            }
        }

        //Get motor temperatures
        double driveTemp = Double.NEGATIVE_INFINITY;
        double rotTemp = Double.NEGATIVE_INFINITY;
        for (SwerveModule module : swerveModules) {
            driveTemp = Math.max(driveTemp, module.getDriveTemperature());
            rotTemp = Math.max(rotTemp, module.getRotationTemperature());
        }
        driveMotorHighestTemp = driveTemp;
        rotationMotorHighestTemp = rotTemp;
    }

    /**
     * Sets whether the robot should be field-oriented
     */
    public void setFieldOriented(boolean fieldOriented, double rotationOffset) {
        this.fieldOriented = fieldOriented;
        gyro.setOffset(rotationOffset);
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
    public void resetPosition() {
        pose = initialPose;
        odometry.setPose(pose);
    }

    /**
     * Sets the initial pose of the swerve (useful for auto)
     */
    public void setInitialPose(Pose2d pose) {
        initialPose = pose;
    }

    /**
     * Sets the brakes on each module
     */
    public void setBrakes(boolean brakesOn) {
        for (SwerveModule module : swerveModules) {
            module.setBrakes(brakesOn);
        }
    }

    /**
     * Get the temperature of the hottest drive motor in degrees Celsius
     */
    public double getDriveTemperature() {
        return driveMotorHighestTemp;
    }

    /**
     * Get the temperature of the hottest rotation motor in degrees Celsius
     */
    public double getRotationTemperature() {
        return rotationMotorHighestTemp;
    }
}
