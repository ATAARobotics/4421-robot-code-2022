package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

  // the 3 initial PID
  private final PIDController xController = new PIDController(0.2, 0, 0);
  private final PIDController yController = new PIDController(0.2, 0, 0);
  private final PIDController rotController = new PIDController(0.14, 0, 0);

  private SwerveOdometry odometry;

  private static final Transform3d TAG_TO_GOAL = new Transform3d(
      new Translation3d(1.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, Math.PI));

  private final SwerveDriveSubsystem swerveDrive;
  private Pose2d targetPose;


  // takes in targetPose and the tolerance it is allowed. rotTolerance(degrees)
  public DriveTagCommand(Pose2d targetPose, double driveTolerance, double rotTolerance, SwerveDriveSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.odometry = swerveDrive.getOdometry();
    this.targetPose = targetPose;

    // stop when values are small
    xController.setTolerance(driveTolerance);
    yController.setTolerance(driveTolerance);
    rotController.setTolerance(Units.degreesToRadians(rotTolerance));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // PID VALUES
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
  }

  @Override
  public void execute() {
    var robotPose = odometry.getPose();

        // Transform the tag's pose to set our goal
        var goalPose = targetPose;

        // Drive
        // Translation2d distanceToTarget=target.getRange;
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());

        SmartDashboard.putNumber("Robot X", robotPose.getX());

        SmartDashboard.putNumber("X-Dist", goalPose.getX() - robotPose.getX());
        SmartDashboard.putNumber("Y-Dist", goalPose.getY() - robotPose.getY());
        SmartDashboard.putNumber("R-Dist", (goalPose.getRotation().getDegrees()));

      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      var rotTemp = robotPose.getRotation().getRadians();
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

      // swerveDrive.setSwerveDrive(xSpeed, ySpeed, rotSpeed, true);
      swerveDrive.setSwerveDrive(xSpeed, ySpeed, rotSpeed, true);

  }
}
