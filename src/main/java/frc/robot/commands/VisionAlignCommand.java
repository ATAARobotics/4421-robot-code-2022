package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightState;

public class VisionAlignCommand extends CommandBase {

    private LimelightSubsystem m_limelightSubsystem;
    private SwerveDriveSubsystem m_swerveDriveSubsystem;

    private ProfiledPIDController visionPID = new ProfiledPIDController(0.6, 0, 0.001, new TrapezoidProfile.Constraints(
            Constants.MAXIMUM_ROTATIONAL_SPEED / 4, Constants.MAXIMUM_ROTATIONAL_ACCELERATION / 2));

    private boolean measurementsComplete = false;
    private double targetAngle;

    private DriveCommand targetingCommand = null;
    private boolean targetingDone = false;

    private double tolerance = 0.05;
    private int onTargetCount = 0;

    public VisionAlignCommand(LimelightSubsystem m_limelightSubsystem, SwerveDriveSubsystem m_swerveDriveSubsystem) {
        addRequirements(m_limelightSubsystem, m_swerveDriveSubsystem);

        this.m_limelightSubsystem = m_limelightSubsystem;
        this.m_swerveDriveSubsystem = m_swerveDriveSubsystem;
        System.out.println("Limelight comand init");
        visionPID.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putString("Limelight State", "Starting");
        visionPID.setTolerance(tolerance);
    }

    @Override
    public void execute() {
        if (!measurementsComplete) {
            LimelightState curState = m_limelightSubsystem.measure();
            System.out.println(curState.toString());
            if (curState == LimelightState.SUCCESS) {
                SmartDashboard.putString("Limelight State", "Messuring success");
                measurementsComplete = true;
                targetAngle = m_limelightSubsystem.getTargetAngle() + m_swerveDriveSubsystem.getHeading();
                targetAngle += Math.PI * 3;
                targetAngle %= Math.PI * 2;
                targetAngle -= Math.PI;

                targetingCommand = new DriveCommand(m_swerveDriveSubsystem, () -> 0, () -> 0,
                        this::getTargetingPIDOutput);

                targetingCommand.schedule();
            } else if (curState == LimelightState.FAILED) {
                SmartDashboard.putString("Limelight State", "Messuring failed");
                targetingDone = true;
                return;
            }
        }

        if (measurementsComplete && visionPID.atSetpoint()) {
            onTargetCount++;
        } else {
            onTargetCount = 0;
        }

        if (onTargetCount >= 10) {
            SmartDashboard.putString("Limelight State", "Completed");
            targetingDone = true;
        }
    }

    @Override
    public boolean isFinished() {
        return targetingDone;
    }

    @Override
    public void end(boolean interrupted) {
        if (targetingCommand != null) {
            targetingCommand.cancel();
        }
    }

    public double getTargetingPIDOutput() {
        double currentAngle = m_swerveDriveSubsystem.getHeading();
        double error = targetAngle - currentAngle;
        return visionPID.calculate(error, 0.0);
    }
}
