package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

public class Square extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final AutoCommand m_autoCommand = new AutoCommand(null, 0);

    public Square(SwerveDriveSubsystem swerveDriveSubsystem) {
        addRequirements(swerveDriveSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addCommands(
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), 0.03, 3, 0.6),
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), 0.03, 3, 0.6)
        );

    }

}