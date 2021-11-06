package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class SwerveModule {
    
    //Restrictions on the minimum and maximum speed of the rotation motors (0 to 1)
    private double maxRotationSpeed = 1.0;
    private double minRotationSpeed = 0.0;

    private TalonFX driveMotor;
    private TalonFX rotationMotor;
    private CANCoder rotationEncoder;

    private double ticksPerMeter;

    //The rotation encoders all have their zero position in a different place, so keep track of how far off zero is from straight ahead
    private double rotationOffset;

    //The right-hand modules have their wheels facing the other way, so we need to invert their direction
    private double inversionConstant = 1.0;

    //The ID number of the module
    private int id;

    //The name of the module - not used for much other than debugging
    private String name;

    //The velocity (-1 to 1) to run the motor
    private double driveVelocity = 0.0;
    private double reverseMultiplier = 1.0;

    //Create a PID for controlling the angle of the module
    private PIDController angleController = new PIDController(0.4, 0.0, 0.001);

    //Create a PID for controlling the velocity of the module
    private PIDController velocityController = new PIDController(0.07, 0.0, 0.001);

    //Safety override
    private boolean cancelAllMotion = false;

    /**
     * Creates a swerve module with the given hardware
     * 
     * @param driveMotor The Talon SRX running the wheel
     * @param rotationMotor The Victor SPX that rotates the wheel
     * @param rotationEncoder The input from the encoder
     * @param rotationOffset The distance from zero that forward is on the encoder
     * @param invertDrive Whether to invert the direction of the wheel
     * @param driveTicksPerMeter The number of encoder ticks per meter on the drive motor
     * @param id The ID of the module
     * @param name The name of the module
     */
    public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANCoder rotationEncoder, double rotationOffset, boolean invertDrive, double driveTicksPerMeter, int id, String name) {
        this.driveMotor = driveMotor;
        this.rotationMotor = rotationMotor;
        this.rotationEncoder = rotationEncoder;
        this.rotationOffset = rotationOffset;

        this.ticksPerMeter = driveTicksPerMeter;

        this.id = id;
        this.name = name;

        if (invertDrive) {
            this.inversionConstant = -1.0;
        }

        //Set up the encoder from the drive motor
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.driveMotor.setSelectedSensorPosition(0);

        //Set up the encoder from the rotation motor
        this.rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        //Because the rotation is on a circle, not a line, we want to take the shortest route to the setpoint - this function tells the PID it is on a circle from 0 to 2*Pi
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * This function should run every teleopPeriodic
     */
    public boolean periodic() {
        //Set the drive velocity
        double calculated = 0.0;
        double velocity = 0.0;
        if (driveVelocity != 0.0 && !cancelAllMotion) {
            calculated = velocityController.calculate(getVelocity()) + (driveVelocity * 0.5);

            velocity = MathUtil.clamp(calculated, -RobotMap.MAX_SAFE_SPEED_OVERRIDE, RobotMap.MAX_SAFE_SPEED_OVERRIDE);

            //DO NOT MESS WITH THIS CODE please
            //thanks
            if (Math.abs(velocity) > RobotMap.MAX_SAFE_SPEED_OVERRIDE) {
                //For some reason, the robot is above the max safe speed - disable the bot
                return true;
            } else {
                driveMotor.set(ControlMode.PercentOutput, velocity * inversionConstant);
            }
        } else {
            driveMotor.set(ControlMode.PercentOutput, 0);
        }

        //Get the rotation velocity
        double rotationVelocity = angleController.calculate(getAngle());
        //Clamp the value (not scale because faster is okay, it's on a PID)
        rotationVelocity = MathUtil.clamp(rotationVelocity, -maxRotationSpeed, maxRotationSpeed);
        if (rotationVelocity > -minRotationSpeed && rotationVelocity < minRotationSpeed) {
            rotationVelocity = 0.0;
        }

        //If the robot isn't moving at all, don't rotate the module
        if (driveVelocity != 0.0 && !cancelAllMotion) {
            //Set the rotation motor velocity based on the next value from the angle PID, clamped to not exceed the maximum speed
            rotationMotor.set(ControlMode.PercentOutput, rotationVelocity);
        } else {
            rotationMotor.set(ControlMode.PercentOutput, 0.0);
        }

        if (RobotMap.DETAILED_MODULE_INFORMATION) {
            SmartDashboard.putNumber(name + " Speed Setpoint", driveVelocity);
            SmartDashboard.putNumber(name + " PID Output", rotationVelocity);
            SmartDashboard.putNumber(name + " PID Error", angleController.getPositionError());
            SmartDashboard.putNumber(name + " Raw Speed", velocity);
            SmartDashboard.putNumber(name + " Speed (m/s)", getVelocity());
            SmartDashboard.putNumber(name + " Angle", getAngle());
            SmartDashboard.putNumber(name + " Angle Target", getTargetAngle());
            SmartDashboard.putNumber(name + " Distance", getDistance(false));
        }

        if (RobotMap.DETAILED_ENCODER_INFORMATION) {
            SmartDashboard.putNumber(name + " Raw Encoder Ticks", driveMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber(name + " Raw Rotation", rotationEncoder.getAbsolutePosition());
        }

        return false;
    }

    /**
     * Sets the velocity to drive the module in meters/second.
     * This can exceed the maximum velocity specified in RobotMap.
     * 
     * @param velocity The velocity to drive the module. 
     */
    public void setDriveVelocity(double velocity) {
        driveVelocity = velocity * reverseMultiplier;
        velocityController.setSetpoint(driveVelocity);
    }

    /**
     * Sets the target in radians for the angle PID
     * 
     * @param angle The angle to try to reach. This value should be between -Pi and Pi
     */
    public void setTargetAngle(double angle) {
        double currentAngle = getAngle();

        //If the smallest angle between the current angle and the target is greater than Pi/2, invert the velocity and turn the wheel to a closer angle
        //Math.atan2(y, x) computes the angle to a given point from the x-axis
        if (Math.abs(Math.atan2(Math.sin(angle - currentAngle), Math.cos(angle - currentAngle))) > Math.PI / 2.0) {            
            angle += Math.PI;
            angle %= 2.0 * Math.PI;
            //Ensure the value is not negative
            if (angle < 0) {
                angle += 2.0 * Math.PI;
            }
            reverseMultiplier = -1.0;
        } else {
            reverseMultiplier = 1.0;
        }

        angleController.setSetpoint(angle);
    }

    /**
     * Get the distance that the drive wheel has turned
     * 
     * @param rawTicks Whether the output should be in raw encoder ticks instead of meters
     */
    public double getDistance(boolean rawTicks) {
        //Raw encoder ticks
        double distance = driveMotor.getSelectedSensorPosition();

        if (!rawTicks) {
            //Meters
            distance /= ticksPerMeter;
        }

        return distance;
    }

    /**
     * Gets the current velocity in meters/second that the drive wheel is moving
     */
    public double getVelocity() {
        //Raw encoder ticks per 100 ms
        double velocity = driveMotor.getSelectedSensorVelocity();

        //Raw encoder ticks per 1 s
        velocity *= 10;

        //Meters per second
        velocity /= ticksPerMeter;

        return -velocity * inversionConstant;
    }

    /**
     * Gets the angle in radians of the module from -Pi to Pi
     */
    public double getAngle() {
        double angle = (rotationEncoder.getAbsolutePosition() / 360 * 2.0 * Math.PI) + rotationOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        angle -= Math.PI;

        return angle;
    }

    /**
     * Gets the target angle in radians from the angle PID
     */
    public double getTargetAngle() {
        return angleController.getSetpoint();
    }

    /**
     * Stops all motion on this module - safety override
     */
    public void stop() {
        cancelAllMotion = true;
    }

    /**
     * Get the id of the module
     */
    public int getId() {
        return this.id;
    }

    /**
     * Get the name of the module
     */
    public String getName() {
        return this.name;
    }
}
