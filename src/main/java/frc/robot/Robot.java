package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;

public class Robot extends TimedRobot {

    // Timer for keeping track of when to disable brakes after being disabled so
    // that the robot stops safely - DO NOT USE COMMANDS-DOES NOT WORK WHEN DISABLED
    private Timer brakesOffTimer = new Timer();

    private RobotContainer robotContainer = null;

    private Command m_autonomousCommand = null;;

    public Robot() {
        robotContainer = new RobotContainer();
        if (!RobotMap.COMP_MODE) {
            DriverStation.silenceJoystickConnectionWarning(true);
        } else {
            DriverStation.silenceJoystickConnectionWarning(false);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (RobotMap.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Controller Temp",
                    robotContainer.getSwerveDriveSubsystem().getDriveTemperature());
            SmartDashboard.putNumber("Rotation Controller Temp",
                    robotContainer.getSwerveDriveSubsystem().getRotationTemperature());
            SmartDashboard.putNumber("Robot Heading", robotContainer.getSwerveDriveSubsystem().getHeading());
        }
    }

    @Override
    public void disabledInit() {
        // Cancel all commands
        CommandScheduler.getInstance().cancelAll();
        // Write remaining blackbox data to file
        Blackbox.getInstance().finishLog();
        // Start brake timer
        brakesOffTimer.reset();
        brakesOffTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (brakesOffTimer.get() > 2.5) {
            brakesOffTimer.stop();
            brakesOffTimer.reset();
            robotContainer.getSwerveDriveSubsystem().setBrakes(false);
        }
    }

    @Override
    public void autonomousInit() {
        Blackbox.getInstance().startLog();
        m_autonomousCommand = robotContainer.getAutonomousChooser().getSelected();
        m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        Blackbox.getInstance().periodic();
    }

    @Override
    public void teleopInit() {
        Blackbox.getInstance().startLog();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
        robotContainer.getSwerveDriveSubsystem().setBrakes(true);

        robotContainer.getIntakeSubsystem().intakeOff();
        robotContainer.getShooterSubsystem().pidReset();
        robotContainer.getShooterSubsystem().setDefaultCommand(new RunCommand(robotContainer.getShooterSubsystem()::shooterHighFar, robotContainer.getShooterSubsystem()));

        if (!RobotMap.FIELD_ORIENTED) {
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(false, 0);
        }

        robotContainer.getSwerveDriveSubsystem().setDefaultCommand(new DriveCommand(robotContainer.getSwerveDriveSubsystem(), robotContainer.getOI()::getXVelocity, robotContainer.getOI()::getYVelocity,
                robotContainer.getOI()::getRotationVelocity, robotContainer.getOI()::getSpeed, () -> 0.8 * robotContainer.getOI().getSpeed()));

        Blackbox.getInstance().addLog("Gyro Reading", robotContainer.getSwerveDriveSubsystem()::getHeading);
        Blackbox.getInstance().addLog("Field Oriented", robotContainer.getSwerveDriveSubsystem()::getFieldOriented);
        Blackbox.getInstance().addLog("Shooter Speed (Primary)", robotContainer.getShooterSubsystem()::getSpeedPrimary);
        Blackbox.getInstance().addLog("Shooter Speed (Secondary)", robotContainer.getShooterSubsystem()::getSpeedSecondary);
        Blackbox.getInstance().addLog("Shooter Near Setpoint", robotContainer.getShooterSubsystem()::nearSetpoint);
    
    }

    @Override
    public void teleopPeriodic() {
        Blackbox.getInstance().periodic();
        robotContainer.getOI().checkInputs();

        if (RobotMap.LASERSHARK_DIAGNOSTICS) {
            robotContainer.getMagazineSubsystem().lasersharkValues();
        }

        if (RobotMap.REPORTING_DIAGNOSTICS) {
            robotContainer.getClimbMotorSubsystem().diagnostic();
            robotContainer.getShooterSubsystem().diagnostic();

            SmartDashboard.putNumber("Joy X", robotContainer.getOI().getXVelocity());
            SmartDashboard.putNumber("Joy Y", robotContainer.getOI().getYVelocity());
            SmartDashboard.putNumber("Rotation", robotContainer.getOI().getRotationVelocity());
            SmartDashboard.putNumber("Slider", robotContainer.getOI().getSpeed());
        }

        if (robotContainer.getOI().getToggleFieldOriented()) {
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(!robotContainer.getSwerveDriveSubsystem().getFieldOriented(), 0);
            robotContainer.getSwerveDriveSubsystem().resetHeading();
        }
    }

    @Override
    public void testInit() {
        Blackbox.getInstance().startLog();
        CommandScheduler.getInstance().cancelAll();
    }
}
