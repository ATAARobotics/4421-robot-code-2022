package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveCommand extends CommandBase {
    private SwerveDriveSubsystem swerveSubsystem;
    private DoubleSupplier xSupplier, ySupplier, rotationSupplier, speedSupplier, rotationSpeedSupplier;
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier, DoubleSupplier rotationSpeedSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.speedSupplier = speedSupplier;
        this.rotationSupplier = rotationSupplier;
        this.rotationSpeedSupplier = rotationSpeedSupplier;
        swerveSubsystem = swerve;

        addRequirements(swerve);
    }

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier) {
        this(swerve, xSupplier, ySupplier, rotationSupplier, speedSupplier, () -> 1);
    }

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {
        this(swerve, xSupplier, ySupplier, rotationSupplier, () -> 1, () -> 1);
    }

    @Override
    public void execute() {
        double xSpeed = xSupplier.getAsDouble();
        if (xSpeed > 0.0) {
            xVelocity = Math.min(xSupplier.getAsDouble() * speedSupplier.getAsDouble(), Constants.MAXIMUM_SPEED);
        } else {
            xVelocity = Math.max(xSupplier.getAsDouble() * speedSupplier.getAsDouble(), -Constants.MAXIMUM_SPEED);
        }

        double ySpeed = ySupplier.getAsDouble();
        if (ySpeed > 0.0) {
            yVelocity = Math.min(ySupplier.getAsDouble() * speedSupplier.getAsDouble(), Constants.MAXIMUM_SPEED);
        } else {
            yVelocity = Math.max(ySupplier.getAsDouble() * speedSupplier.getAsDouble(), -Constants.MAXIMUM_SPEED);
        }

        double rotationSpeed = rotationSupplier.getAsDouble();
        if (rotationSpeed > 0.0) {
            rotationVelocity = Math.min(rotationSupplier.getAsDouble() * rotationSpeedSupplier.getAsDouble(), Constants.MAXIMUM_ROTATIONAL_SPEED);
        } else {
            rotationVelocity = Math.max(rotationSupplier.getAsDouble() * rotationSpeedSupplier.getAsDouble(), -Constants.MAXIMUM_ROTATIONAL_SPEED);
        }


        swerveSubsystem.setSwerveDrive(xVelocity, yVelocity, rotationVelocity, true);
    }

}