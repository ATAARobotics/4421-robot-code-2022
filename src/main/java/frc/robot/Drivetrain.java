// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final boolean COMP_BOT = false;
  public static final double[] TICKS_PER_METER = COMP_BOT ? new double[] {
    43191.3003, 43777.7504, 43744.6686, 42909.4215
  }
      : new double[] {
      0, 0, 0, 0
    };

  public static final int[] DRIVE_MOTORS_ID = { 1, 2, 3, 4 };
  public static final int[] ROTATION_MOTORS_ID = { 5, 6, 7, 8 };
  public static final int[] ROTATION_ENCODERS_ID = { 9, 10, 11, 12 };
  public static final int PIGEON_ID = 20;
    // Swerve offset
  public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
    2.1138, -0.3758, -2.1506, 0.4740
  }
            : new double[] {
                  0, 0, 0, 0
          };
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  
//   SwerveModule frontLeftModule = new SwerveModule(driveMotors[0], rotationMotors[0],
//   new CANCoder(Constants.ROTATION_ENCODERS_ID[0], bus), Constants.ANGLE_OFFSET[0], true,
//   Constants.TICKS_PER_METER[0], 0, "Front Left");
// SwerveModule frontRightModule = new SwerveModule(driveMotors[1], rotationMotors[1],
//   new CANCoder(Constants.ROTATION_ENCODERS_ID[1], bus), Constants.ANGLE_OFFSET[1], false,
//   Constants.TICKS_PER_METER[1], 1, "Front Right");
// SwerveModule rearLeftModule = new SwerveModule(driveMotors[2], rotationMotors[2],
//   new CANCoder(Constants.ROTATION_ENCODERS_ID[2], bus), Constants.ANGLE_OFFSET[2], true,
//   Constants.TICKS_PER_METER[2], 2, "Rear Left");
// SwerveModule rearRightModule = new SwerveModule(driveMotors[3], rotationMotors[3],
//   new CANCoder(Constants.ROTATION_ENCODERS_ID[3], bus), Constants.ANGLE_OFFSET[3], false,
//   Constants.TICKS_PER_METER[3], 3, "Rear Right");

  private final SwerveModule m_frontLeft = new SwerveModule(DRIVE_MOTORS_ID[0], ROTATION_MOTORS_ID[0], 
    new CANCoder(ROTATION_ENCODERS_ID[0], "canivore"), ANGLE_OFFSET[0], true,
    TICKS_PER_METER[0], 0, "Front Left");
  private final SwerveModule m_frontRight = new SwerveModule(DRIVE_MOTORS_ID[1], ROTATION_MOTORS_ID[1],
    new CANCoder(ROTATION_ENCODERS_ID[0], "canivore"), ANGLE_OFFSET[1], true,
    TICKS_PER_METER[1], 0, "Front Right");

  private final SwerveModule m_backLeft = new SwerveModule(DRIVE_MOTORS_ID[2], ROTATION_MOTORS_ID[2], 
    new CANCoder(ROTATION_ENCODERS_ID[0], "canivore"), ANGLE_OFFSET[2], true,
    TICKS_PER_METER[2], 0, "Back Left");

  private final SwerveModule m_backRight = new SwerveModule(DRIVE_MOTORS_ID[3], ROTATION_MOTORS_ID[3],     
    new CANCoder(ROTATION_ENCODERS_ID[0], "canivore"), ANGLE_OFFSET[3], true,
    TICKS_PER_METER[3], 0, "Back Right");


  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    // System.out.println(swerveModuleStates[0].speedMetersPerSecond);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
