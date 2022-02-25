package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class ClimbTwoCommand extends SequentialCommandGroup {
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private SwerveDrive m_swerveSubsystem;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public ClimbTwoCommand(ClimbArmSubsystem climbArmSubsystem, ClimbMotorSubsystem climbMotorSubsystem, SwerveDrive swerveDrive) {
        m_climbArmSubsystem = climbArmSubsystem;
        m_climbMotorSubsystem = climbMotorSubsystem;
        m_swerveSubsystem = swerveDrive;
        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem, m_swerveSubsystem);
        addCommands(
                new InstantCommand(m_climbArmSubsystem::armVertical), 
                new RunCommand(m_climbMotorSubsystem::climberUp).withInterrupt(m_climbMotorSubsystem::climberMax),
                new InstantCommand(m_climbMotorSubsystem::climberStop),
                new RunCommand(() -> swerveDrive.setSwerveDrive(0 * RobotMap.MAXIMUM_SPEED, 
                0.5 * RobotMap.MAXIMUM_SPEED, 
                0 * RobotMap.MAXIMUM_ROTATIONAL_SPEED)).withTimeout(1/3),
                new InstantCommand(() -> swerveDrive.setSwerveDrive(0, 0, 0)),
                new RunCommand(m_climbMotorSubsystem::climberDown).withInterrupt(m_climbMotorSubsystem::climberMin),
                new InstantCommand(m_climbMotorSubsystem::climberStop)
        );
    }
}