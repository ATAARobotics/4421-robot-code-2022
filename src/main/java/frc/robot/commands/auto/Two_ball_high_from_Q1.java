package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.*;

public class Two_ball_high_from_Q1 extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;


    public Two_ball_high_from_Q1(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, MagazineSubsystem magazineSubsystem, ShooterSubsystem shooterSubsystem, AutoPaths autoPaths) {
        addRequirements(swerveDriveSubsystem, intakeSubsystem, hoodSubsystem, shooterSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_magazineSubsystem = magazineSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addCommands(
                new InstantCommand(m_intakeSubsystem::intakeOn, m_intakeSubsystem),
                new InstantCommand(m_shooterSubsystem::shooterHighFar, m_shooterSubsystem),
                new AutoDriveCommand(m_swerveDriveSubsystem, autoPaths.GetQuadrant1LeftBall2(), true),
                new WaitCommand(1),
                new InstantCommand(m_intakeSubsystem::intakeOff, m_intakeSubsystem),
                new WaitCommand(1),
                new InstantCommand(m_magazineSubsystem::magazineOn, m_magazineSubsystem),
                new WaitCommand(3),
                new InstantCommand(m_magazineSubsystem::magazineOff, m_magazineSubsystem),
                new InstantCommand(m_shooterSubsystem::shooterOff, m_shooterSubsystem)
        );
    }

}