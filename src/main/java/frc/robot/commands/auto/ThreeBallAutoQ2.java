package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.*;

public class ThreeBallAutoQ2 extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;
    private AutoPaths autoPaths;

    public ThreeBallAutoQ2(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, MagazineSubsystem magazineSubsystem, ShooterSubsystem shooterSubsystem) {
        addRequirements(swerveDriveSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_magazineSubsystem = magazineSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(m_shooterSubsystem::shooterAuto), 
                        new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem)), 
                new InstantCommand(m_intakeSubsystem::intakeOn, m_intakeSubsystem),
                new AutoDriveCommand(m_swerveDriveSubsystem,autoPaths.getQuadrant2EdgeBall5()),
                new WaitUntilCommand(m_magazineSubsystem::bothDetectors),
                new RunCommand(m_magazineSubsystem::magazineOn).withInterrupt(m_magazineSubsystem.getEmptyMagazineTrigger()),
                new AutoDriveCommand(m_swerveDriveSubsystem,autoPaths.getBall5Ball4()),
                new WaitUntilCommand(m_magazineSubsystem::topDetectorOnly),
                new RunCommand(m_magazineSubsystem::magazineOn).withInterrupt(m_magazineSubsystem.getEmptyMagazineTrigger()),
                new AutoDriveCommand(m_swerveDriveSubsystem,autoPaths.getBall4Ball13())
        );
    }
}