package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class ClimbThreeCommand extends SequentialCommandGroup {
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private SwerveDrive m_swerveSubsystem;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public ClimbThreeCommand(ClimbArmSubsystem climbArmSubsystem, ClimbMotorSubsystem climbMotorSubsystem, SwerveDrive swerveDrive) {
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        m_swerveSubsystem = swerveDrive;
        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem, m_swerveSubsystem);
        addCommands(
            new ClimbTwoCommand(m_climbArmSubsystem, m_climbMotorSubsystem, m_swerveSubsystem),
            new SwingCommand(m_climbArmSubsystem, m_climbMotorSubsystem)

        );
    }
}