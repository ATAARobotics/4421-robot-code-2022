// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BetterJoystick;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.wpilibj.XboxController;

public class AprilTagLimelight<Transform3d> extends SubsystemBase {  
  private static final double CAMERA_HEIGHT_METERS = 0.7;
  private static final double CAMERA_PITCH_RADIANS = 0;
  private static final double TARGET_HEIGHT_METERS = 0;
  // Change this to match the name of your camera
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
      List<TargetCorner> corners = target.getCorners();
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

          // Use this range as the measurement we give to the PID controller.
          // -1.0 required to ensure positive PID controller effort _increases_ range
          // forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
