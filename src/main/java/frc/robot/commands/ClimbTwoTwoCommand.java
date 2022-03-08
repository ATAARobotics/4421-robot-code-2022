package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbTwoTwoCommand extends SequentialCommandGroup {
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;

    /**
     * Second step in climbing - pulls arm all the way down
     */
    public ClimbTwoTwoCommand(ClimbArmSubsystem climbArmSubsystem, ClimbMotorSubsystem climbMotorSubsystem) {
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem);
        addCommands(
                new RunCommand(m_climbMotorSubsystem::climberDown).withInterrupt(m_climbMotorSubsystem::climberMin),
                new InstantCommand(m_climbMotorSubsystem::climberStop)
        );
    }
}