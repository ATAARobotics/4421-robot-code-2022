package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectBallCommand extends ParallelCommandGroup {
    private final ShooterSubsystem m_shooter;
    private final MagazineSubsystem m_magazine;
    private final IntakeSubsystem m_intake;

    /**
     * A command to eject a single ball out the top of the robot in case a ball is stuck
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public EjectBallCommand(ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem, IntakeSubsystem intakeSubsystem) {
        m_shooter = shooterSubsystem;
        m_magazine = magazineSubsystem;
        m_intake = intakeSubsystem;
        addCommands(
            new StartEndCommand(m_shooter::shooterReverse, m_shooter::shooterOff, m_shooter),
            new StartEndCommand(m_magazine::magazineReverse, m_magazine::magazineOff, m_magazine),
            new StartEndCommand(m_intake::intakeReverse, m_intake::intakeOff, m_intake)
        );
    }
}