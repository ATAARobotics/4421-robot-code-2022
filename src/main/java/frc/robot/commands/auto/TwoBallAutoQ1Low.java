package frc.robot.commands.auto;
/* 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand; 

import frc.robot.AutoPaths; */

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class TwoBallAutoQ1Low extends SequentialCommandGroup {
    /*
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem; */

    public TwoBallAutoQ1Low(SwerveDriveSubsystem swerveDriveSubsystem, ClimbMotorSubsystem climbMotorSubsystem, ClimbArmSubsystem climbArmSubsystem, IntakeSubsystem intakeSubsystem, HoodSubsystem hoodSubsystem, MagazineSubsystem magazineSubsystem, ShooterSubsystem shooterSubsystem) {
        addRequirements(swerveDriveSubsystem, climbMotorSubsystem, climbArmSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem);
        /*
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        m_shooterSubsystem = shooterSubsystem;
        m_magazineSubsystem = magazineSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_intakeSubsystem = intakeSubsystem; */
        addCommands(
        );
    }
}