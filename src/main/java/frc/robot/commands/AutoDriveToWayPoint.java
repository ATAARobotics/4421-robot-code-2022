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

public class AutoDriveToWayPoint extends CommandBase {

    private SwerveDriveSubsystem m_swerveDriveSubsystem;
    private SwerveOdometry odometry;

    // Poses
    private Pose2d targetPose;
    private Pose2d robotPose;
    private Pose2d goalPose;

    // speed variables
    private double xSpeed;
    private double ySpeed;
    private double rotTemp;
    private double rotSpeed;
    private double speedLimit;
    private double rotLimit;

    // PID
    private final PIDController xController = new PIDController(1.0, 0.1, 0);
    private final PIDController yController = new PIDController(1.0, 0.1, 0);
    private final PIDController rotController = new PIDController(2.0, 0, 0);

    public AutoDriveToWayPoint(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose, double driveTolerance, double rotTolerance, double speedLimit, double rotLimit) {
        this.m_swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPose = targetPose;
        this.odometry = swerveDriveSubsystem.getOdometry();
        xController.setTolerance(driveTolerance);
        yController.setTolerance(driveTolerance);
        rotController.setTolerance(Units.degreesToRadians(rotTolerance));
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.speedLimit = speedLimit;
        this.rotLimit = rotLimit;
        addRequirements(this.m_swerveDriveSubsystem);
    }

    public AutoDriveToWayPoint(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose) {
      this(swerveDriveSubsystem, targetPose, Constants.DTOLERANCE, Constants.RTOLERANCE, Constants.SPEEDLIMIT, Constants.ROTLIMIT);
    }

    @Override
    public void initialize() {
        m_swerveDriveSubsystem.setBrakes(true);
    }

    @Override
    public void execute() {

      robotPose = odometry.getPose();
      goalPose = targetPose;
      
      xController.setSetpoint(goalPose.getX());
      yController.setSetpoint(goalPose.getY());
      rotController.setSetpoint(goalPose.getRotation().getRadians());

      SmartDashboard.putNumber("X-Goal", goalPose.getX());
      SmartDashboard.putNumber("Y-Goal", goalPose.getY());
      SmartDashboard.putNumber("Rot-Goal", goalPose.getRotation().getRadians());
      
      SmartDashboard.putNumber("robotPoseX", robotPose.getX());
      SmartDashboard.putNumber("robotPoseY", robotPose.getY());
      SmartDashboard.putNumber("robotPoseR", robotPose.getRotation().getRadians());

      // var xSpeed = Math.clamp(xController.calculate(robotPose.getX()), speedLimit);
      xSpeed = MathUtil.clamp(xController.calculate(robotPose.getX()), -speedLimit, speedLimit);
      if (xController.atSetpoint()) {
        System.out.println("X ACHIEVED");
        xSpeed = 0;
      }

      ySpeed = MathUtil.clamp(yController.calculate(robotPose.getY()), -speedLimit, speedLimit);
      if (yController.atSetpoint()) {
        System.out.println("Y ACHIEVED");
        ySpeed = 0;
      }

      rotTemp = m_swerveDriveSubsystem.getHeading();
      if(rotTemp > 0) {
        rotTemp = rotTemp - Math.PI;
      }
      else {
        rotTemp = rotTemp + Math.PI;
      }

      rotSpeed = MathUtil.clamp(-rotController.calculate(rotTemp), -rotLimit, rotLimit);
      if (rotController.atSetpoint()) {
        System.out.println("ROT ACHIEVED");
        rotSpeed = 0;
      }

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // x and y is flipped
      m_swerveDriveSubsystem.setSwerveDrive(ySpeed, xSpeed, rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
