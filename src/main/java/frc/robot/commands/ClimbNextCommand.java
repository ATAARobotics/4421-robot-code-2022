package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class ClimbNextCommand extends SequentialCommandGroup {
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;

    /**
     * Climbs up to the next rung, if the robot is already on a rung
     */
    public ClimbNextCommand(ClimbMotorSubsystem climbMotorSubsystem, ClimbArmSubsystem climbArmSubsystem) {
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem);
        addCommands(
                new WaitUntilCommand(m_climbArmSubsystem::armEngaged),
                new InstantCommand(m_climbMotorSubsystem::climberMaxSpeed),
                new RunCommand(m_climbMotorSubsystem::climberUp).until(m_climbMotorSubsystem::climberMid),
                new InstantCommand(m_climbArmSubsystem::armTilt), 
                new RunCommand(m_climbMotorSubsystem::climberUp).until(m_climbMotorSubsystem::climberMax),
                new InstantCommand(m_climbArmSubsystem::armVertical),
                new InstantCommand(m_climbMotorSubsystem::climberSlowSpeed),
                new RunCommand(m_climbMotorSubsystem::climberDown).withTimeout(3),
                new InstantCommand(m_climbMotorSubsystem::climberNormalSpeed),
                new RunCommand(m_climbMotorSubsystem::climberDown).until(m_climbMotorSubsystem::climberMin),
                new InstantCommand(m_climbMotorSubsystem::climberStop),
                new InstantCommand(() -> System.out.println("DONE"))
        );
    }
}