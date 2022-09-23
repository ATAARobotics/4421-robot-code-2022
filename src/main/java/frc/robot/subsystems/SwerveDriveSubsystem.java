package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pigeon;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.SwerveOdometry;

public class SwerveDriveSubsystem extends SubsystemBase {

    private Pigeon pigeon;

    // Whether the swerve should be field-oriented
    boolean fieldOriented = false;

    // An array of all the modules on the swerve drive
    private SwerveModule[] swerveModules;

    // The odometry for the swerve drive
    private SwerveOdometry odometry;

    // The current pose of the robot
    private Pose2d pose;

    // The position that the robot started at
    private Pose2d initialPose;

    private double driveMotorHighestTemp = 0;
    private double rotationMotorHighestTemp = 0;

    // Safety speed override, this *shouldn't* ever be true
    private boolean safetyDisable = false;

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private double[] velocities;
    private double[] angles;

    /**
     * Set up the swerve drive
     * 
     * @param gyro        The gyro object running on the robot
     * @param initialPose The initial pose that the robot is in
     */
    public SwerveDriveSubsystem(Pigeon pigeon, Translation2d initialPosition, String bus) {
        this.pigeon = pigeon;
        this.initialPose = new Pose2d(initialPosition, new Rotation2d(0.0));
        TalonFX[] driveMotors = { new TalonFX(Constants.DRIVE_MOTORS_ID[0], bus),
                new TalonFX(Constants.DRIVE_MOTORS_ID[1], bus), new TalonFX(Constants.DRIVE_MOTORS_ID[2], bus),
                new TalonFX(Constants.DRIVE_MOTORS_ID[3], bus) };
        TalonFX[] rotationMotors = { new TalonFX(Constants.ROTATION_MOTORS_ID[0], bus),
                new TalonFX(Constants.ROTATION_MOTORS_ID[1], bus), new TalonFX(Constants.ROTATION_MOTORS_ID[2], bus),
                new TalonFX(Constants.ROTATION_MOTORS_ID[3], bus) };

        // Initialize four swerve modules using the SwerveModule class
        SwerveModule frontLeftModule = new SwerveModule(driveMotors[0], rotationMotors[0],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[0], bus), Constants.ANGLE_OFFSET[0], true,
                Constants.TICKS_PER_METER[0], 0, "Front Left");
        SwerveModule frontRightModule = new SwerveModule(driveMotors[1], rotationMotors[1],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[1], bus), Constants.ANGLE_OFFSET[1], false,
                Constants.TICKS_PER_METER[1], 1, "Front Right");
        SwerveModule rearLeftModule = new SwerveModule(driveMotors[2], rotationMotors[2],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[2], bus), Constants.ANGLE_OFFSET[2], true,
                Constants.TICKS_PER_METER[2], 2, "Rear Left");
        SwerveModule rearRightModule = new SwerveModule(driveMotors[3], rotationMotors[3],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[3], bus), Constants.ANGLE_OFFSET[3], false,
                Constants.TICKS_PER_METER[3], 3, "Rear Right");

        // Put the swerve modules in an array so we can process them easier
        swerveModules = new SwerveModule[] {
                frontLeftModule,
                frontRightModule,
                rearLeftModule,
                rearRightModule
        };

        // Set up odometry
        odometry = new SwerveOdometry(initialPose);

        // Initialize the pose
        pose = initialPose;
    }

    /**
     * This function should be run during every teleop and auto periodic
     */
    public void setSwerveDrive(double xVelocity, double yVelocity, double rotationVelocity) {
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotationVelocity = rotationVelocity;

    }

    @Override
    public void periodic() {
        pigeon.update();
        double gyroAngle = getHeading();

        if (fieldOriented) {
            double originalX = this.xVelocity;
            double originalY = this.yVelocity;

            this.xVelocity = originalX * Math.cos(-gyroAngle) - originalY * Math.sin(-gyroAngle);
            this.yVelocity = originalY * Math.cos(-gyroAngle) + originalX * Math.sin(-gyroAngle);
        }

        // Get the wheelbase and track width from RobotMap. These are important because
        // a long rectangular robot turns differently than a square robot
        double wheelbase = Constants.WHEELBASE;
        double trackWidth = Constants.TRACK_WIDTH;

        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("X Velocity", xVelocity);
            SmartDashboard.putNumber("Y Velocity", yVelocity);
            SmartDashboard.putNumber("Rotation Velocity", rotationVelocity);
        }

        // Calculate wheel velocities and angles
        double a, b, c, d;

        a = xVelocity - rotationVelocity * wheelbase / 2;
        b = this.xVelocity + rotationVelocity * wheelbase / 2;
        c = this.yVelocity - rotationVelocity * trackWidth / 2;
        d = this.yVelocity + rotationVelocity * trackWidth / 2;

        velocities = new double[] {
                Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2)),
                Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2)),
                Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2)),
                Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2))
        };
        angles = new double[] {
                // Math.atan2(y, x) computes the angle to a given point from the x axis
                Math.atan2(b, c),
                Math.atan2(b, d),
                Math.atan2(a, c),
                Math.atan2(a, d)
        };

        if (!safetyDisable) {
            if (Constants.REPORTING_DIAGNOSTICS) {
                SmartDashboard.putNumber("Gyro Value", pigeon.getYaw());
            }

            // Execute functions on each swerve module
            for (SwerveModule module : swerveModules) {
                // Set the drive velocity in meters/second for the module
                module.setDriveVelocity(getModuleVelocity(module.getId()));

                // Set module angle target in radians from -Pi to Pi
                module.setTargetAngle(getModuleAngle(module.getId()));

                // Run periodic tasks on the module (running motors)
                if (module.periodic()) {
                    // Something has gone horribly wrong if this code is running, there are several
                    // checks to prevent it
                    // Abort due to excessive speed
                    safetyDisable = true;
                    break;
                }
            }

            // Update the current pose with the latest velocities, angle, and a timestamp
            boolean useOdometry = false;
            if (useOdometry) {
                pose = odometry.update(getXVelocity(), getYVelocity(), pigeon.getYaw(), Timer.getFPGATimestamp());
            }

            if (Constants.REPORTING_DIAGNOSTICS) {
                SmartDashboard.putNumber("Distance X", odometry.getPose().getX());
                SmartDashboard.putNumber("Distance Y", odometry.getPose().getY());
            }
        } else {
            for (SwerveModule module : swerveModules) {
                module.stop();
            }
            DriverStation.reportError("DANGER: MODULES DISABLED FOR SAFETY", false);
        }

        // Get motor temperatures
        if (Constants.REPORTING_DIAGNOSTICS) {
            double driveTemp = Double.NEGATIVE_INFINITY;
            double rotTemp = Double.NEGATIVE_INFINITY;
            for (SwerveModule module : swerveModules) {
                driveTemp = Math.max(driveTemp, module.getDriveTemperature());
                rotTemp = Math.max(rotTemp, module.getRotationTemperature());
            }
            driveMotorHighestTemp = driveTemp;
            rotationMotorHighestTemp = rotTemp;
        }
    }

    /**
     * Sets whether the robot should be field-oriented
     */
    public void setFieldOriented(boolean fieldOriented, double currentYaw) {
        this.fieldOriented = fieldOriented;
        pigeon.setYaw(currentYaw);
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
        return pigeon.getYaw();
    }

    /**
     * Resets the robot heading
     */
    public void resetHeading() {
        pigeon.setYaw(0);
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

    /**
     * Gets the velocity of a specific module in meters/second
     * 
     * @param moduleId The ID of the module to get
     */
    public double getModuleVelocity(int moduleId) {
        return velocities[moduleId];
    }

    /**
     * Gets the angle that the module should be set to
     * 
     * @param moduleId The ID of the module to get
     */
    public double getModuleAngle(int moduleId) {
        return angles[moduleId];
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getRotationVelocity() {
        return rotationVelocity;
    }
}
