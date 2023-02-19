package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

public class Square extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    public Square(SwerveDriveSubsystem swerveDriveSubsystem) {
        addRequirements(swerveDriveSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addCommands(
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(2, 0, new Rotation2d(0)), 0.03, 3, 0.6),
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(2, 2, new Rotation2d(0)), 0.03, 3, 0.6),
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(0, 2, new Rotation2d(0)), 0.03, 3, 0.6),
                new AutoDriveToWayPoint(swerveDriveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), 0.03, 3, 0.6)
        );

    }

}