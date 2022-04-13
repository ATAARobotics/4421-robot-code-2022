package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class DriveCommand extends CommandBase {
    private SwerveDrive swerveSubsystem;
    private DoubleSupplier xSupplier, ySupplier, rotationSupplier, speedSupplier, rotationSpeedSupplier;
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    public DriveCommand(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier, DoubleSupplier rotationSpeedSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.speedSupplier = speedSupplier;
        this.rotationSupplier = rotationSupplier;
        this.rotationSpeedSupplier = rotationSpeedSupplier;
        swerveSubsystem = swerve;

        addRequirements(swerve);
    }

    public DriveCommand(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.speedSupplier = speedSupplier;
        this.rotationSpeedSupplier = () -> 1;
        swerveSubsystem = swerve;

        addRequirements(swerve);
    }

    public DriveCommand(SwerveDrive swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.speedSupplier = () -> 1;
        this.rotationSpeedSupplier = () -> 1;
        swerveSubsystem = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        xVelocity = xSupplier.getAsDouble()*speedSupplier.getAsDouble()*RobotMap.MAXIMUM_SPEED;
        yVelocity = ySupplier.getAsDouble()*speedSupplier.getAsDouble()*RobotMap.MAXIMUM_SPEED;       
        rotationVelocity = rotationSupplier.getAsDouble()*rotationSpeedSupplier.getAsDouble()*RobotMap.MAXIMUM_ROTATIONAL_SPEED;
        swerveSubsystem.setSwerveDrive(xVelocity,yVelocity,rotationVelocity);
    }

}