package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.VisionConstants;

public class DrivePlaceCommand extends CommandBase {

  // the 3 initial PID
  private final PIDController xController = new PIDController(1.0, 0.1, 0);
  private final PIDController yController = new PIDController(1.0, 0.1, 0);
  private final PIDController rotController = new PIDController(2.0, 0, 0);

  private SwerveOdometry odometry;

  private final SwerveDriveSubsystem swerveDrive;
  private Pose2d targetPose;

  // takes in targetPose and the tolerance it is allowed. rotTolerance(degrees)
  public DrivePlaceCommand(Pose2d targetPose, double driveTolerance, double rotTolerance, SwerveDriveSubsystem swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.odometry = swerveDrive.getOdometry();
    this.targetPose = targetPose;

    // stop when values are small
    xController.setTolerance(driveTolerance);
    yController.setTolerance(driveTolerance);
    rotController.setTolerance(Units.degreesToRadians(rotTolerance));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // // PID VALUES
    SmartDashboard.putNumber("X-P", 1.0);
    SmartDashboard.putNumber("X-I", 0.1);
    SmartDashboard.putNumber("X-D", 0.0);

    SmartDashboard.putNumber("Y-P", 1.0);
    SmartDashboard.putNumber("Y-I", 0.1);
    SmartDashboard.putNumber("Y-D", 0.0);

    SmartDashboard.putNumber("R-P", 2.0);
    SmartDashboard.putNumber("R-I", 0.0);
    SmartDashboard.putNumber("R-D", 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    
    double newxp = SmartDashboard.getNumber("X-P", 1.0);
    double newxi = SmartDashboard.getNumber("X-I", 0.1);
    double newxd = SmartDashboard.getNumber("X-D", 0.0);
    xController.setPID(newxp, newxi, newxd);

    double newyp = SmartDashboard.getNumber("Y-P", 1.0);
    double newyi = SmartDashboard.getNumber("Y-I", 0.1);
    double newyd = SmartDashboard.getNumber("Y-D", 0.0);
    yController.setPID(newyp, newyi, newyd);

    double newrp = SmartDashboard.getNumber("R-P", 2.0);
    double newri = SmartDashboard.getNumber("R-I", 0.0);
    double newrd = SmartDashboard.getNumber("R-D", 0.0);
    rotController.setPID(newrp, newri, newrd);

    rotController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    var robotPose = odometry.getPose();

        // Transform the tag's pose to set our goal
        var goalPose = targetPose;

        
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());

        SmartDashboard.putNumber("X-Goal", goalPose.getX());
        SmartDashboard.putNumber("Y-Goal", goalPose.getY());
        SmartDashboard.putNumber("Rot-Goal", goalPose.getRotation().getRadians());

        SmartDashboard.putNumber("X-Dist", goalPose.getX() - robotPose.getX());
        SmartDashboard.putNumber("Y-Dist", goalPose.getY() - robotPose.getY());
        SmartDashboard.putNumber("R-Dist", (goalPose.getRotation().getRadians() - swerveDrive.getHeading()));

      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atSetpoint()) {
        xSpeed = 0;
        System.out.println("X ACHIEVED");
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atSetpoint()) {
        ySpeed = 0;
        System.out.println("Y ACHIEVED");
      }

      var rotTemp = swerveDrive.getHeading();

      var rotSpeed = -(rotController.calculate(rotTemp));
      if (rotController.atSetpoint()) {
        rotSpeed = 0;
        System.out.println("ROT ACHIEVED");
      }

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // yspeed = xspeed  // x speed = y speed. setSwerveDrive is wrong
      swerveDrive.setSwerveDrive(0, 0, rotSpeed, true);

  }

  @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
