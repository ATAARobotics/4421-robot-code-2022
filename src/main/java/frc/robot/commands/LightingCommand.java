package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.MagazineSubsystem;

public class LightingCommand extends CommandBase {
    private final LightingSubsystem m_lightingSubsystem;
    private final BooleanSupplier topLasershark;
    private final BooleanSupplier bottomLasershark;

    /**
     * A command to index a ball into the top of the magazine if there is no ball already present
     * @param magazineSubsystem The magazine subsystem this command will run on.
     */
    public LightingCommand(BooleanSupplier topLasershark, BooleanSupplier bottomLasershark, LightingSubsystem lightingSubsystem) {
        m_lightingSubsystem = lightingSubsystem;
        this.topLasershark = topLasershark;
        this.bottomLasershark = bottomLasershark;
        addRequirements(m_lightingSubsystem);
    }

    @Override
    public void execute() {
        if (topLasershark.getAsBoolean() && bottomLasershark.getAsBoolean()) {
            m_lightingSubsystem.lightUpGreen();
        }
        else if (topLasershark.getAsBoolean() || bottomLasershark.getAsBoolean()) {
            m_lightingSubsystem.lightUpYellow();
        }
        else {
            m_lightingSubsystem.lightUpRed();
        }
    }
}