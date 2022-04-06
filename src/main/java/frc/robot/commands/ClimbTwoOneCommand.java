package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbTwoOneCommand extends SequentialCommandGroup {
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ClimbArmSubsystem m_climbArmSubsystem;

    /**
     * First step in climbing - flips arm to vertical and extends to maximum
     */
    public ClimbTwoOneCommand(ClimbMotorSubsystem climbMotorSubsystem, ClimbArmSubsystem climbArmSubsystem) {
        m_climbMotorSubsystem = climbMotorSubsystem;
        m_climbArmSubsystem = climbArmSubsystem;
        addRequirements(m_climbMotorSubsystem, m_climbArmSubsystem);
        addCommands(
                new InstantCommand(m_climbArmSubsystem::armVertical),
                new InstantCommand(m_climbMotorSubsystem::climberMaxSpeed),
                new RunCommand(m_climbMotorSubsystem::climberUp).until(m_climbMotorSubsystem::climberMax),
                new InstantCommand(m_climbMotorSubsystem::climberNormalSpeed),
                new InstantCommand(m_climbMotorSubsystem::climberStop)
        );
    }
}