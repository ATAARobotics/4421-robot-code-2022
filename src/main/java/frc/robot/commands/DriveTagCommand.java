package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagLimelight;
import frc.robot.subsystems.SwerveDrive;

public class DriveTagCommand extends CommandBase{
    

    // velocity and acceleration constraints
    private static final TrapezoidProfile.Constraints POS_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);


    private final ProfiledPIDController posController = new ProfiledPIDController(0, 0, 0, POS_CONSTRAINTS);
    private final ProfiledPIDController rotController = new ProfiledPIDController(0, 0, 0, ROT_CONSTRAINTS);

    private final PhotonCamera photonCamera;
    private final SwerveDrive swerveDrive;
    private final Supplier<Pose2d> poseProvider;

    private PhotonTrackedTarget lastTarget;

    public DriveTagCommand(PhotonCamera photonCamera, SwerveDrive swerveDrive, Supplier<Pose2d> poseProvider) {
        this.photonCamera = photonCamera;
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;

        // stop when values are small
        posController.setTolerance(0.2);
        rotController.setTolerance(Units.degreesToRadians(3));

        SmartDashboard.setDefaultBoolean("Target Visible", false);

        addRequirements(swerveDrive);
    }

    @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    rotController.reset(robotPose.getRotation().getRadians());
    // there is type error check out later
    posController.reset(robotPose.getTranslation());
    
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();

    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream().findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        posController.setGoal(goalPose.getTranslation());
        rotController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
    //   drivetrainSubsystem.stop();
        SmartDashboard.putBoolean("Target Visible", false);

    } else {
      // Drive to the target
    //   var xSpeed = xController.calculate(robotPose.getX());
    //   if (xController.atGoal()) {
    //     xSpeed = 0;
    //   }

    //   var ySpeed = yController.calculate(robotPose.getY());
    //   if (yController.atGoal()) {
    //     ySpeed = 0;
    //   }

    `SmartDashboard.putBoolean("Target Visible", true);

        var posSpeed = posController.calculate(robotPose2d.getTranslation().getDistance(AprilTagLimelight.getDistanceValue()));    

        var rotSpeed = rotController.calculate(robotPose2d.getRotation().getRadians());
        if (rotController.atGoal()) {
            rotSpeed = 0;
        }

        // Use Trig
        swerveDrive.setSwerveDrive(Math.cos((double) posSpeed), Math.sin((double) posSpeed), rotSpeed);
        // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, robotPose2d.getRotation()));
    }
  }
}
