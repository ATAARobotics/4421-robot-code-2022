package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class SwingCommand extends SequentialCommandGroup {
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public SwingCommand(ClimbArmSubsystem climbArmSubsystem, ClimbMotorSubsystem climbMotorSubsystem) {
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem);
        addCommands(
                new RunCommand(m_climbMotorSubsystem::climberUp).withInterrupt(m_climbMotorSubsystem::climberMid),
                new InstantCommand(m_climbArmSubsystem::armTilt), 
                new RunCommand(m_climbMotorSubsystem::climberUp).withInterrupt(m_climbMotorSubsystem::climberMax),
                new RunCommand(m_climbMotorSubsystem::climberDown).withInterrupt(m_climbMotorSubsystem::climberMin),
                new InstantCommand(m_climbMotorSubsystem::climberStop)
        );
    }
}