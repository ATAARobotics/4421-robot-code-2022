// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.List;

import javax.crypto.spec.PSource;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BetterJoystick;
import frc.robot.Constants;
import frc.robot.SwerveOdometry;

import org.photonvision.RobotPoseEstimator;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.wpilibj.XboxController;


public class AprilTagLimelight extends SubsystemBase {  
  private static final double CAMERA_HEIGHT_METERS = 0.7;
  private static final double CAMERA_PITCH_RADIANS = 0;
  private static final double TARGET_HEIGHT_METERS = 0;
  private static final double SAFETY_OFFSET = 0.6;
  private double forwardSpeed = 0;
  Gyro gyro;
  // Change this to match the name of your camera
  double range;
  Pose2d robotPose;
  PhotonCamera camera = new PhotonCamera("Limelight");
  PhotonCamera photonCamera;
  frc.robot.AprilTag aprilTagPos;

  // odometry
  SwerveOdometry odometry;
  
  public AprilTagLimelight(SwerveOdometry odometry) {
    super();

    this.range = 0.0d;
    this.odometry = odometry;
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();
    // Get the current best target.
    
    int targetID = -1;

    if (hasTargets == true) {
      // Get information from target.
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      //  Transform2d pose = target.getCameraToTarget();
      List<TargetCorner> corners = target.getDetectedCorners();
      targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      //  Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      //  Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    
    // Vision-alignment mode
    // Query the latest result 2from PhotonVision
    result = camera.getLatestResult();

      robotPose = odometry.getPose();
        // First calculate range
        double range = PhotonUtils.calculateDistanceToTargetMeters(
                          CAMERA_HEIGHT_METERS,
                          TARGET_HEIGHT_METERS,
                          CAMERA_PITCH_RADIANS,
                          Units.degreesToRadians(result.getBestTarget().getPitch()));
                          double GOAL_RANGE_METERS = range-SAFETY_OFFSET;
                          // Use this range as the measurement we give to the PID controller.
            
        this.range = range; 



        // Calculate a translation from the camera to the target.
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation( //what we want
          range, Rotation2d.fromDegrees(-target.getYaw()));

        double kTargetPitch = target.getPitch();
        double kTargetHeight = TARGET_HEIGHT_METERS;
        edu.wpi.first.math.geometry.Transform3d cameraToRobot = new edu.wpi.first.math.geometry.Transform3d();
        Pose3d aprilTagFieldLayout = new Pose3d();
      
        
        // Calculate robot's field relative pose
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, cameraToRobot);
        
        //Rotation2d targetYaw = PhotonUtils.getYawToPose3d(robotPose, targetPose);
        System.out.println("Robot Pose X: " + robotPose.getX());
        System.out.println("Robot Pose Y: " + robotPose.getY());
        System.out.println("Robot Pose Rot: " + robotPose.getRotation());

        // adds the position of robot to april tag to find the actual position
        if (targetID >= 1 && targetID <= 8) {
          aprilTagPos = Constants.VisionConstants.AprilTagPos[targetID-1];
          Pose2d tempPose = getActualPose(robotPose.toPose2d(), aprilTagPos.aprilTagPose.toPose2d());

        }
    }
    }

  public Pose2d getActualPose(Pose2d robot, Pose2d april) {
    double x, y, rot;
    x = robot.getX() + april.getX();
    y = robot.getY() + april.getY();
    rot = robot.getRotation().getRadians() + april.getRotation().getRadians();
    Pose2d newPose = new Pose2d(x,y,new Rotation2d(rot));
    return newPose;
  }

  public double getRange() {
    return this.range-0.6;
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
