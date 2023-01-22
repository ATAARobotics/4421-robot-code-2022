package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.VisionConstants;

public class DriveTagCommand extends CommandBase {

  // velocity and acceleration constraints
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(0.8, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(0.8, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController rotController = new ProfiledPIDController(0.8, 0, 0, ROT_CONSTRAINTS);

  private SwerveOdometry odometry;

  private static final Transform3d TAG_TO_GOAL = new Transform3d(
      new Translation3d(1.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final SwerveDriveSubsystem swerveDrive;

  private PhotonTrackedTarget lastTarget;

  public DriveTagCommand(PhotonCamera photonCamera, SwerveDriveSubsystem swerveDrive) {
    this.photonCamera = photonCamera;
    this.swerveDrive = swerveDrive;
    this.odometry = swerveDrive.getOdometry();

    // stop when values are small
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotController.setTolerance(Units.degreesToRadians(1));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.setDefaultBoolean("Target Visible", false);
    SmartDashboard.setDefaultNumber("X-P", 0.8);
    SmartDashboard.putNumber("X-P", 0.8);
    SmartDashboard.setDefaultNumber("X-I", 0.0);
    SmartDashboard.putNumber("X-I", 0.0);
    SmartDashboard.setDefaultNumber("X-D", 0.0);
    SmartDashboard.putNumber("X-D", 0.0);

    SmartDashboard.setDefaultNumber("Y-P", 0.8);
    SmartDashboard.putNumber("Y-P", 0.8);
    SmartDashboard.setDefaultNumber("Y-I", 0.0);
    SmartDashboard.putNumber("Y-I", 0.0);
    SmartDashboard.setDefaultNumber("Y-D", 0.0);
    SmartDashboard.putNumber("Y-D", 0.0);

    SmartDashboard.setDefaultNumber("R-P", 0.8);
    SmartDashboard.putNumber("R-P", 0.8);
    SmartDashboard.setDefaultNumber("R-I", 0.0);
    SmartDashboard.putNumber("R-I", 0.0);
    SmartDashboard.setDefaultNumber("R-D", 0.0);
    SmartDashboard.putNumber("R-D", 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    
    double newxp = SmartDashboard.getNumber("X-P", 0.8);
    double newxi = SmartDashboard.getNumber("X-I", 0.0);
    double newxd = SmartDashboard.getNumber("X-D", 0.0);
    xController.setPID(newxp, newxi, newxd);

    double newyp = SmartDashboard.getNumber("Y-P", 0.8);
    double newyi = SmartDashboard.getNumber("Y-I", 0.0);
    double newyd = SmartDashboard.getNumber("Y-D", 0.0);
    yController.setPID(newyp, newyi, newyd);

    double newrp = SmartDashboard.getNumber("R-P", 0.8);
    double newri = SmartDashboard.getNumber("R-I", 0.0);
    double newrd = SmartDashboard.getNumber("R-D", 0.0);
    rotController.setPID(newrp, newri, newrd);

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
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    var photonRes = photonCamera.getLatestResult();

    var target = photonRes.getBestTarget();
      
      if (target != null && target.getPoseAmbiguity() < 0.6) {
        System.out.println("target found");
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
        // Translation2d distanceToTarget=target.getRange;
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        rotController.setGoal(goalPose.getRotation().getRadians());

        SmartDashboard.putBoolean("Target Visible", true);

        SmartDashboard.putNumber("X-Dist", robotPose.getX());
        SmartDashboard.putNumber("Y-Dist", robotPose.getY());
        SmartDashboard.putNumber("R-Dist", Units.radiansToDegrees(robotPose.getRotation().getAngle()));

      var xSpeed = -xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = -yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var rotSpeed = -rotController.calculate(robotPose2d.getRotation().getRadians());
      if (rotController.atGoal()) {
        rotSpeed = 0;
      }

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      double newX = xSpeed * Math.cos(-goalPose.getRotation().getRadians()) - xSpeed * Math.sin(-goalPose.getRotation().getRadians());
      double newY = ySpeed * Math.cos(-goalPose.getRotation().getRadians()) + ySpeed * Math.sin(-goalPose.getRotation().getRadians());

      // swerveDrive.setSwerveDrive(xSpeed, ySpeed, rotSpeed, true);
      swerveDrive.setSwerveDrive(newY, 0, 0, true);

    } else {
        swerveDrive.setSwerveDrive(0, 0, 0, true);
    }

  }
}
