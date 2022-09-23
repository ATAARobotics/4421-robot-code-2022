package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveCommand extends CommandBase {
    private Trajectory trajectory;
    private SwerveDriveSubsystem m_swerveDriveSubsystem;
    private Timer timer = new Timer();
    private ProfiledPIDController rotationController = new ProfiledPIDController(0.9, 0, 0.001,
            new TrapezoidProfile.Constraints(Constants.MAXIMUM_ROTATIONAL_SPEED,
                    Constants.MAXIMUM_ROTATIONAL_ACCELERATION));
    private State desiredState;
    private Pose2d desiredPose;
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    public AutoDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, Trajectory trajectory) {
        this.trajectory = trajectory;
        xVelocity = 0;
        yVelocity = 0;
        rotationVelocity = 0;
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // Configure the rotation PID to take the shortest route to the setpoint
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.reset(
                new TrapezoidProfile.State(trajectory.getStates().get(0).poseMeters.getRotation().getRadians(), 0.0));
    }

    @Override
    public void execute() {
        desiredState = trajectory.sample(timer.get());

        // Get the current position of the robot
        Pose2d currentPose = m_swerveDriveSubsystem.getPose();

        // Get the position we want to be at
        desiredPose = desiredState.poseMeters;

        // Get the current angle of the robot
        double currentAngle = currentPose.getRotation().getRadians();
        double desiredAngle = desiredPose.getRotation().getRadians();

        // Get the total speed the robot should be travelling (not accounting for
        // deviations)
        double totalVelocity = desiredState.velocityMetersPerSecond;

        // Get the velocity in the X and Y direction based on the heading and total
        // speed
        xVelocity = totalVelocity * desiredPose.getRotation().getCos();
        yVelocity = totalVelocity * desiredPose.getRotation().getSin();

        // Get the current rotational velocity from the rotation PID based on the
        // desired angle
        rotationVelocity = rotationController.calculate(currentAngle, desiredAngle);
        m_swerveDriveSubsystem.setSwerveDrive(
                xVelocity,
                -yVelocity,
                rotationVelocity);
    }

    @Override
    public boolean isFinished() {
        return desiredState == trajectory.getStates().get(trajectory.getStates().size() - 1);
    }
}
