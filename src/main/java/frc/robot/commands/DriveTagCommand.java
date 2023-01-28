package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController xController = new PIDController(0.2, 0, 0);
  private final PIDController yController = new PIDController(0.2, 0, 0);
  private final PIDController rotController = new PIDController(0.14, 0, 0);

  private SwerveOdometry odometry;

  private static final Transform3d TAG_TO_GOAL = new Transform3d(
      new Translation3d(1.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final SwerveDriveSubsystem swerveDrive;
  private PhotonTrackedTarget target;

  public DriveTagCommand(PhotonCamera photonCamera, SwerveDriveSubsystem swerveDrive) {
    this.photonCamera = photonCamera;
    this.swerveDrive = swerveDrive;
    this.odometry = swerveDrive.getOdometry();

    // stop when values are small
    xController.setTolerance(0.03);
    yController.setTolerance(0.03);
   // rotController.setTolerance(Units.degreesToRadians(1));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.setDefaultBoolean("Target Visible", false);
    SmartDashboard.setDefaultNumber("X-P", 0.2);
    SmartDashboard.putNumber("X-P", 0.2);
    SmartDashboard.setDefaultNumber("X-I", 0.0);
    SmartDashboard.putNumber("X-I", 0.0);
    SmartDashboard.setDefaultNumber("X-D", 0.0);
    SmartDashboard.putNumber("X-D", 0.0);

    SmartDashboard.setDefaultNumber("Y-P", 0.2);
    SmartDashboard.putNumber("Y-P", 0.2);
    SmartDashboard.setDefaultNumber("Y-I", 0.0);
    SmartDashboard.putNumber("Y-I", 0.0);
    SmartDashboard.setDefaultNumber("Y-D", 0.0);
    SmartDashboard.putNumber("Y-D", 0.0);

    SmartDashboard.setDefaultNumber("R-P", 0.14);
    SmartDashboard.putNumber("R-P", 0.14);
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

    rotController.reset();
    // there is type error check out later
    xController.reset();
    yController.reset();

    var photonRes = photonCamera.getLatestResult();

    this.target = photonRes.getBestTarget();
  }

  @Override
  public void execute() {
    var robotPose2d = odometry.getPose();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));


      
      if (target != null && target.getPoseAmbiguity() < 0.6) {
        // This is new target data, so recalculate the goal

        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(VisionConstants.ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);

        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        // Translation2d distanceToTarget=target.getRange;
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());

        SmartDashboard.putBoolean("Target Visible", true);

        SmartDashboard.putNumber("Robot X", robotPose.getX());

        SmartDashboard.putNumber("X-Dist", goalPose.getX() - robotPose.getY());
        SmartDashboard.putNumber("Y-Dist", goalPose.getY() - robotPose.getX());
        SmartDashboard.putNumber("R-Dist", (goalPose.getRotation().getDegrees()));
        SmartDashboard.putNumber("X-Goal", goalPose.getX());
        SmartDashboard.putNumber("Y-Goal", goalPose.getY());
        SmartDashboard.putNumber("R-Goal", (goalPose.getRotation().getDegrees()));

      var xSpeed = -xController.calculate(robotPose.getX());
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed = -yController.calculate(robotPose.getY());
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      var rotTemp = robotPose2d.getRotation().getRadians();
      if(rotTemp > 0) {
        rotTemp = rotTemp - Math.PI;
      }
      else {
        rotTemp = rotTemp + Math.PI;
      }

      var rotSpeed = (rotController.calculate(-rotTemp));
      if (rotController.atSetpoint()) {
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
