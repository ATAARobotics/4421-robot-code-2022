package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    
    // Create objects to run auto and teleop code
    public Teleop teleop = null;

    // Timer for keeping track of when to disable brakes after being disabled so
    // that the robot stops safely
    private Timer brakesTimer = new Timer();
    private boolean brakesTimerCompleted = false;
    private final RobotContainer robotContainer;

    private Command m_autonomousCommand;

    public Robot() {
        robotContainer = new RobotContainer();
        // Controller objects
        teleop = new Teleop(robotContainer.getSwerveDriveSubsystem(), robotContainer.getClimbMotorSubsystem(), robotContainer.getClimbArmSubsystem(), robotContainer.getIntakeSubsystem(), robotContainer.getHoodSubsystem(), robotContainer.getMagazineSubsystem(), robotContainer.getShooterSubsystem());
    }

    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive

        // Turn off the brakes
        robotContainer.getSwerveDriveSubsystem().setBrakes(false);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (RobotMap.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Controller Temp", robotContainer.getSwerveDriveSubsystem().getDriveTemperature());
            SmartDashboard.putNumber("Rotation Controller Temp", robotContainer.getSwerveDriveSubsystem().getRotationTemperature());
        }
    }

    @Override
    public void disabledInit() {
        // Cancel all commands
        CommandScheduler.getInstance().cancelAll();

        // Write remaining blackbox data to file
        Blackbox.getInstance().finishLog();

        // Reset and start the brakes timer
        brakesTimerCompleted = false;
        brakesTimer.reset();
        brakesTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (!brakesTimerCompleted && brakesTimer.get() > 2) {
            // Turn off the brakes
            robotContainer.getSwerveDriveSubsystem().setBrakes(false);
            brakesTimerCompleted = true;
        }
    }

    @Override
    public void autonomousInit() {
        Blackbox.getInstance().startLog();
        m_autonomousCommand = robotContainer.getAutonomousChooser().getSelected();
        m_autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        Blackbox.getInstance().startLog();
        m_autonomousCommand.cancel();
        teleop.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        teleop.teleopPeriodic();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testInit() {
        Blackbox.getInstance().startLog();
        CommandScheduler.getInstance().cancelAll();
    }
}
