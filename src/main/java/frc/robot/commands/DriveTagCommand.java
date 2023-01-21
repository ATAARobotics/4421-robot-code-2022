package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.AprilTagLimelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.VisionConstants;

public class DriveTagCommand extends CommandBase{
    

    // velocity and acceleration constraints
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController rotController = new ProfiledPIDController(0.1, 0, 0, ROT_CONSTRAINTS);

    private SwerveOdometry odometry;

    private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

    private final AprilTagLimelight aprilTagLimelight;
    private final PhotonCamera photonCamera;
    private final SwerveDriveSubsystem swerveDrive;

    private PhotonTrackedTarget lastTarget;

    public DriveTagCommand(PhotonCamera photonCamera, SwerveDriveSubsystem swerveDrive, AprilTagLimelight aprilTagLimelight) {
        this.photonCamera = photonCamera;
        this.swerveDrive = swerveDrive;
        this.aprilTagLimelight = aprilTagLimelight;
        this.odometry = swerveDrive.getOdometry();

        // stop when values are small
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        rotController.setTolerance(Units.degreesToRadians(3));
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.setDefaultBoolean("Target Visible", false);

        addRequirements(swerveDrive);
    }

    @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = odometry.getPose();
    rotController.reset(robotPose.getRotation().getRadians());
    // there is type error check out later
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    
  }

  @Override
  public void execute() {
    var robotPose2d = odometry.getPose();
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
        var cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        //Translation2d distanceToTarget=target.getRange;
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
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

    SmartDashboard.putBoolean("Target Visible", true);

    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }
 

    var rotSpeed = rotController.calculate(robotPose2d.getRotation().getRadians());
    if (rotController.atGoal()) {
        rotSpeed = 0;
    }

    swerveDrive.setSwerveDrive(xSpeed, ySpeed, rotSpeed, true);
    }
  }
}
