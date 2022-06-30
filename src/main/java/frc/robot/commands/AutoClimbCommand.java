package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;

public class AutoClimbCommand extends CommandBase {

    private ClimbMotorSubsystem m_climbMotorSubsystem;
    private ClimbArmSubsystem m_climbArmSubsystem;

    private Trigger confirmNextStage;
    private Trigger abortClimb;

    private int lastSuccessfulLevel = 0;
    private boolean climbing = false;

    private SequentialCommandGroup executing = null;

    public AutoClimbCommand(ClimbArmSubsystem m_climbArmSubsystem, ClimbMotorSubsystem m_climbMotorSubsystem,
            Trigger confirmNextStage, Trigger abortClimb) {
        this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_climbMotorSubsystem = m_climbMotorSubsystem;
        this.confirmNextStage = confirmNextStage;
        this.abortClimb = abortClimb;

        addRequirements(m_climbArmSubsystem, m_climbMotorSubsystem);
    }

    @Override
    public void initialize() {
        m_climbArmSubsystem.armVertical();
        m_climbMotorSubsystem.setMaxSpeed();
    }

    @Override
    public void execute() {
        if (!climbing && (confirmNextStage.get() || lastSuccessfulLevel == 0)) {
            if (lastSuccessfulLevel == 0) {
                climbing = true;
                executing = new SequentialCommandGroup(
                        new RunCommand(m_climbMotorSubsystem::up).withInterrupt(m_climbMotorSubsystem::atMax),
                        new InstantCommand(m_climbMotorSubsystem::stop),
                        new InstantCommand(() -> {
                            climbing = false;
                            lastSuccessfulLevel = 1;
                        }));
            } else if (lastSuccessfulLevel == 1) {
                climbing = true;
                executing = new SequentialCommandGroup(
                        new InstantCommand(m_climbMotorSubsystem::setNormalSpeed),
                        new RunCommand(m_climbMotorSubsystem::down).withInterrupt(m_climbMotorSubsystem::atMin),
                        new InstantCommand(m_climbMotorSubsystem::stop),
                        new WaitUntilCommand(m_climbArmSubsystem::engaged),
                        new InstantCommand(m_climbMotorSubsystem::resetElevatorEncoder),
                        new InstantCommand(() -> {
                            climbing = false;
                            lastSuccessfulLevel = 2;
                        }));
            } else if (lastSuccessfulLevel == 2 || lastSuccessfulLevel == 3) {
                climbing = true;
                executing = new SequentialCommandGroup(
                        new InstantCommand(m_climbMotorSubsystem::setNormalSpeed),
                        new RunCommand(m_climbMotorSubsystem::up).withInterrupt(m_climbMotorSubsystem::atMid),
                        new InstantCommand(m_climbArmSubsystem::armTilt),
                        new InstantCommand(m_climbMotorSubsystem::setMaxSpeed),
                        new RunCommand(m_climbMotorSubsystem::up).withInterrupt(m_climbMotorSubsystem::atMax),
                        new InstantCommand(m_climbMotorSubsystem::stop),
                        new InstantCommand(m_climbArmSubsystem::armVertical),
                        new InstantCommand(m_climbMotorSubsystem::setSlowSpeed),
                        new RunCommand(m_climbMotorSubsystem::down).withInterrupt(() -> !m_climbArmSubsystem.engaged()),
                        new RunCommand(m_climbMotorSubsystem::down).withTimeout(3),
                        new InstantCommand(m_climbMotorSubsystem::setNormalSpeed),
                        new RunCommand(m_climbMotorSubsystem::down).withInterrupt(m_climbMotorSubsystem::atMin),
                        new InstantCommand(m_climbMotorSubsystem::stop),
                        new WaitUntilCommand(m_climbArmSubsystem::engaged),
                        new InstantCommand(m_climbMotorSubsystem::resetElevatorEncoder),
                        new InstantCommand(() -> {
                            climbing = false;
                            lastSuccessfulLevel++;
                        }));
            }

            executing.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return lastSuccessfulLevel == 4 || abortClimb.get();
    }

    @Override
    public void end(boolean interrupted) {
        if (executing != null) {
            executing.cancel();
        }
        m_climbMotorSubsystem.stop();
    }
}
