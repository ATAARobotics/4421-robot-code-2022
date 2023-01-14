// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import javax.crypto.spec.PSource;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BetterJoystick;
import org.photonvision.RobotPoseEstimator;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.wpilibj.XboxController;

public class AprilTagLimelight<Transform3d> extends SubsystemBase {  
  private static final double CAMERA_HEIGHT_METERS = 0.7;
  private static final double CAMERA_PITCH_RADIANS = 0;
  private static final double TARGET_HEIGHT_METERS = 0;
  Gyro gyro;
  // Change this to match the name of your camera
  Pose2d robotPose;
  Pose2d targetPose;
  PhotonCamera camera = new PhotonCamera("Limelight");
  BetterJoystick xboxController;

  
  AprilTagLimelight(BetterJoystick joysticka) {
    xboxController = joysticka;
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();
    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();
    

    if (hasTargets = true) {
      // Get information from target.
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      //  Transform2d pose = target.getCameraToTarget();
      List<TargetCorner> corners = target.getDetectedCorners();
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      //  Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      //  Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    }
    
    if (xboxController.getButton("B")) {
      // Vision-alignment mode
      // Query the latest result 2from PhotonVision
      result = camera.getLatestResult();

      if (result.hasTargets()) {
          // First calculate range
          double range = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));
                            
                          
        double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
          
          

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        // forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
        // Calculate a translation from the camera to the target.
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
        range, Rotation2d.fromDegrees(-target.getYaw()));
        double kTargetPitch = target.getPitch();
        double kTargetHeight = TARGET_HEIGHT_METERS;
        edu.wpi.first.math.geometry.Transform3d cameraToRobot = new edu.wpi.first.math.geometry.Transform3d();
        Pose3d aprilTagFieldLayout = new Pose3d();
        // Calculate robot's field relative pose
        // Calculate robot's field relative pose
        
        
        // Calculate robot's field relative pose\
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, cameraToRobot);
        System.out.println(robotPose.getX());
        //Rotation2d targetYaw = PhotonUtils.getYawToPose3d(robotPose, targetPose);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
