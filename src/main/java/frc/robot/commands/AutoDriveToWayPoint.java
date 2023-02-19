package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveToWayPoint extends CommandBase {
    private AutoCommand autoCommand;
    private SwerveDriveSubsystem m_swerveDriveSubsystem;
    private State desiredState;
    private Pose2d targetPose;
    private double speedLimitPercentage;

    // PID
    private final PIDController xController = new PIDController(1.0, 0.1, 0);
    private final PIDController yController = new PIDController(1.0, 0.1, 0);
    private final PIDController rotController = new PIDController(2.0, 0, 0);

    public AutoDriveToWayPoint(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose, double driveTolerance, double rotTolerance, double speedLimitPercentage) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        xController.setTolerance(driveTolerance);
        yController.setTolerance(driveTolerance);
        rotController.setTolerance(Units.degreesToRadians(rotTolerance));
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {

        m_swerveDriveSubsystem.setBrakes(true);
        m_swerveDriveSubsystem.setFieldOriented(true, 0);
        m_swerveDriveSubsystem.resetPosition();
    }

    @Override
    public void execute() {

        var robotPose = m_swerveDriveSubsystem.getPose();

        // Transform the tag's pose to set our goal
        var goalPose = targetPose;

        
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());

      var xSpeed = xController.calculate(robotPose.getX()) * speedLimitPercentage;
      if (xController.atSetpoint()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY()) * speedLimitPercentage;
      if (yController.atSetpoint()) {
        ySpeed = 0;
      }

      var rotTemp = robotPose.getRotation().getRadians() * speedLimitPercentage;
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

      // Drive // x and y is flipped
      m_swerveDriveSubsystem.setSwerveDrive(ySpeed, xSpeed, rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
