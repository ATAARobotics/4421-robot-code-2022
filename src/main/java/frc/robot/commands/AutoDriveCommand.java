package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveCommand extends CommandBase {
    private AutoCommand autoCommand;
    private SwerveDriveSubsystem m_swerveDriveSubsystem;
    private Timer timer = new Timer();
    private State desiredState;
    private Pose2d desiredPose;
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean FirstAuto;
    public AutoDriveCommand(SwerveDriveSubsystem swerveDriveSubsystem, AutoCommand autoCommand, boolean firstauto) {
        this.autoCommand = autoCommand;
        xVelocity = 0;
        yVelocity = 0;
        rotationVelocity = 0;
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        FirstAuto = firstauto;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        // Configure the rotation PID to take the shortest route to the setpoint
    }

    @Override
    public void execute() {
        if(FirstAuto){
            m_swerveDriveSubsystem.setBrakes(true);
            RobotContainer.rotationController.reset(new TrapezoidProfile.State(autoCommand.getRotationOffset(), 0.0));

        }
        desiredState = autoCommand.getState(timer.get());
        // Get the current position of the robot
        Pose2d currentPose = m_swerveDriveSubsystem.getPose();

        // Get the position we want to be at
        desiredPose = desiredState.poseMeters;

        // Get the current angle of the robot
        double currentAngle = currentPose.getRotation().getRadians();;
        double desiredAngle = autoCommand.getTargetAngle();

        // Get the total speed the robot should be travelling (not accounting for
        // deviations)
        double totalVelocity = desiredState.velocityMetersPerSecond;

        // Get the velocity in the X and Y direction based on the heading and total
        // speed
        xVelocity = totalVelocity * desiredPose.getRotation().getCos();
        yVelocity = totalVelocity * desiredPose.getRotation().getSin();

        // Get the current rotational velocity from the rotation PID based on the
        // desired angle
        rotationVelocity = RobotContainer.rotationController.calculate(currentAngle, desiredAngle);
        System.out.println(rotationVelocity);
        m_swerveDriveSubsystem.setSwerveDrive(
                xVelocity,
                -yVelocity,
                rotationVelocity);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 1;
    }
}
