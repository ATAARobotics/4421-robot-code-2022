package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.MagazineSubsystem;

public class IndexCommand extends SequentialCommandGroup {
    private final MagazineSubsystem m_magazine;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public IndexCommand(MagazineSubsystem magazineSubsystem) {
        m_magazine = magazineSubsystem;
        addRequirements(m_magazine);
        addCommands(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(m_magazine::magazineOn),
                    new WaitUntilCommand(() -> !m_magazine.bottomDetector()),
                    new InstantCommand(m_magazine::magazineOff)
                ),
                new InstantCommand(m_magazine::magazineOff),
            m_magazine::bottomDetectorOnly)
        );
    }
}