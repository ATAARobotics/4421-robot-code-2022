package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.*;

public class Straight extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;


    public Straight(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, AutoPaths autoPaths) {
        addRequirements(swerveDriveSubsystem, intakeSubsystem, hoodSubsystem, shooterSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addCommands(
                new AutoDriveCommand(swerveDriveSubsystem, autoPaths.getStraight(), true ),
                new AutoDriveCommand(swerveDriveSubsystem, autoPaths.getStraight1(), false )
        );

    }

}