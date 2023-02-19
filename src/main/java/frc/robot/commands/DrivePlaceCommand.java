package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DrivePlaceCommand extends CommandBase {

  // constants
  private static final double XP = 1.0;
  private static final double XI = 1.0;
  private static final double YP = 1.0;
  private static final double YI = 1.0;
  private static final double RP = 2.0;

  // speed variables
  private double xSpeed;
  private double ySpeed;
  private double rotTemp;
  private double rotSpeed;
  private double speedLimit;
  private double rotLimit;

  // the 3 initial PID
  private final PIDController xController = new PIDController(XP, XI, 0);
  private final PIDController yController = new PIDController(YP, YI, 0);
  private final PIDController rotController = new PIDController(RP, 0, 0);

  private SwerveOdometry odometry;

  private final SwerveDriveSubsystem swerveDrive;
  private Pose2d targetPose;

  // takes in targetPose and the tolerance it is allowed. rotTolerance(degrees)
  public DrivePlaceCommand(SwerveDriveSubsystem swerveDrive, Pose2d targetPose, double driveTolerance, double rotTolerance, double speedLimit, double rotLimit) {
    this.swerveDrive = swerveDrive;
    this.odometry = swerveDrive.getOdometry();
    this.targetPose = targetPose;
    this.speedLimit = speedLimit;
    this.rotLimit = rotLimit;

    // stop when values are small
    xController.setTolerance(driveTolerance);
    yController.setTolerance(driveTolerance);
    rotController.setTolerance(Units.degreesToRadians(rotTolerance));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // // PID VALUES
    SmartDashboard.putNumber("X-P", XP);
    SmartDashboard.putNumber("X-I", XI);
    SmartDashboard.putNumber("X-D", 0.0);

    SmartDashboard.putNumber("Y-P", YP);
    SmartDashboard.putNumber("Y-I", YI);
    SmartDashboard.putNumber("Y-D", 0.0);

    SmartDashboard.putNumber("R-P", RP);
    SmartDashboard.putNumber("R-I", 0.0);
    SmartDashboard.putNumber("R-D", 0.0);

    addRequirements(swerveDrive);
  }

  public DrivePlaceCommand(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose) {
    this(swerveDriveSubsystem, targetPose, Constants.DTOLERANCE, Constants.RTOLERANCE, Constants.SPEEDLIMIT, Constants.ROTLIMIT);
  }

  @Override
  public void initialize() {
    
    double newxp = SmartDashboard.getNumber("X-P", XP);
    double newxi = SmartDashboard.getNumber("X-I", XI);
    double newxd = SmartDashboard.getNumber("X-D", 0.0);
    xController.setPID(newxp, newxi, newxd);

    double newyp = SmartDashboard.getNumber("Y-P", 1.0);
    double newyi = SmartDashboard.getNumber("Y-I", YI);
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

      xSpeed = MathUtil.clamp(xController.calculate(robotPose.getX()), -speedLimit, speedLimit);
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      ySpeed = MathUtil.clamp(yController.calculate(robotPose.getY()), -speedLimit, speedLimit);
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      rotTemp = swerveDrive.getHeading();

      rotSpeed = MathUtil.clamp(-rotController.calculate(rotTemp), -rotLimit, rotLimit);
      if (rotController.atSetpoint()) {
        rotSpeed = 0;
      }

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // yspeed = xspeed  // x speed = y speed. setSwerveDrive is wrong
      swerveDrive.setSwerveDrive(ySpeed, xSpeed, rotSpeed, true);

  }

  @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
