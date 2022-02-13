package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MagazineSubsystem;

public class Index extends SequentialCommandGroup {
    private final MagazineSubsystem m_magazine;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public Index(MagazineSubsystem magazineSubsystem) {
        m_magazine = magazineSubsystem;
        addRequirements(m_magazine);
        addCommands(
            new ConditionalCommand(new InstantCommand(m_magazine::magazineOn), 
                    new InstantCommand(m_magazine::magazineOff), 
                    m_magazine::bottomDetectorOnly)
        );
    }
}