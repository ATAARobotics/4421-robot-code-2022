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
            //Wait until the arm has engaged - if the arm is already engaged, this will be a no-op
            new WaitUntilCommand(m_climbArmSubsystem::armEngaged),
            //Set the arm to max speed - we are clamped down on the bar, so we can release as fast as we want
            new InstantCommand(m_climbMotorSubsystem::climberMaxSpeed),
            //Extend the arm to the mid position
            new RunCommand(m_climbMotorSubsystem::climberUp).until(m_climbMotorSubsystem::climberMid),
            //Tip the arm back
            new InstantCommand(m_climbArmSubsystem::armTilt),
            //Extend the arm to maximum
            new RunCommand(m_climbMotorSubsystem::climberUp).until(m_climbMotorSubsystem::climberMax),
            //Stop the arm
            new InstantCommand(m_climbMotorSubsystem::climberStop),
            //Bring the arm back in contact with the next bar
            new InstantCommand(m_climbArmSubsystem::armVertical),
            //Release the passive clamps to allow us to leave the bar
            new InstantCommand(m_climbArmSubsystem::releaseClamps),
            //Set the climb arm to the slow speed to prevent swing
            new InstantCommand(m_climbMotorSubsystem::climberSlowSpeed),
            //Bring the climb arm down a little to get us off the bar
            new RunCommand(m_climbMotorSubsystem::climberDown).withTimeout(3),
            //Set the climb arm to the normal speed, we can go up decently quick
            new InstantCommand(m_climbMotorSubsystem::climberNormalSpeed),
            //Bring the arm all the way down to get us up onto the bar
            new RunCommand(m_climbMotorSubsystem::climberDown).until(m_climbMotorSubsystem::climberMin),
            //Stop the motor - we *should* be on the bar now with the passive hooks in places
            new InstantCommand(m_climbMotorSubsystem::climberStop)
        );
    }
}