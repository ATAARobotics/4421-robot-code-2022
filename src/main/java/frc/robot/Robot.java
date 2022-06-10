package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.auto.ThreeBallAutoQ2;
import frc.robot.commands.auto.TwoBallAutoQ1High;
import frc.robot.commands.auto.TwoBallAutoQ1HighStarve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import edu.wpi.first.math.geometry.Translation2d;

public class Robot extends TimedRobot {
    // Create hardware objects
    private Gyro gyro = null;
    private SwerveDriveSubsystem swerveDriveSubsystem = null;
    private ClimbMotorSubsystem climbMotorSubsystem = null;
    private ClimbArmSubsystem climbArmSubsystem = null;
    private HoodSubsystem hoodSubsystem = null;
    private ShooterSubsystem shooterSubsystem = null;
    private DigitalInput canivoreSwitch = new DigitalInput(6);
    private Limelight limelight = null;

    // Create objects to run auto and teleop code
    public Teleop teleop = null;

    // Timer for keeping track of when to disable brakes after being disabled so
    // that the robot stops safely
    private Timer brakesTimer = new Timer();
    private boolean brakesTimerCompleted = false;

    // The initial position of the robot relative to the field. This is measured
    // from the left-hand corner of the field closest to the driver, from the
    // driver's perspective
    public Translation2d initialPosition = new Translation2d(0, 0);

    // Auto selector on SmartDashboard
    private Command m_autonomousCommand;
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private IntakeSubsystem intakeSubsystem;
    private MagazineSubsystem magazineSubsystem;
    private IndexCommand indexer;

    public Robot() {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gyro = new Gyro();
        gyro.initializeNavX();
        String bus = "rio";
        System.out.println("rio");
        if (canivoreSwitch.get()) {
            bus = "canivore";
            System.out.println("CANivore");
        }
        swerveDriveSubsystem = new SwerveDriveSubsystem(gyro, initialPosition, bus);
        climbMotorSubsystem = new ClimbMotorSubsystem();
        climbArmSubsystem = new ClimbArmSubsystem();
        hoodSubsystem = new HoodSubsystem();
        shooterSubsystem = new ShooterSubsystem(bus);
        intakeSubsystem = new IntakeSubsystem();
        magazineSubsystem = new MagazineSubsystem();
        indexer = new IndexCommand(magazineSubsystem);
        limelight = new Limelight();

        // Controller objects
        teleop = new Teleop(swerveDriveSubsystem, climbMotorSubsystem, climbArmSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem, limelight, gyro);

        // Auto picker
        autoChooser.setDefaultOption("3 Ball Auto (Q2)", new ThreeBallAutoQ2(swerveDriveSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem));
        autoChooser.addOption("High 2 Ball Auto (Q1)", new TwoBallAutoQ1High(swerveDriveSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem));
        autoChooser.addOption("High 2 Ball Auto + Starvation (Q1)", new TwoBallAutoQ1HighStarve(swerveDriveSubsystem, climbMotorSubsystem, climbArmSubsystem, intakeSubsystem, hoodSubsystem, magazineSubsystem, shooterSubsystem));
        autoChooser.addOption("DO NOTHING", new WaitCommand(0));

        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive

        // Put the auto picker on SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Turn off the brakes
        swerveDriveSubsystem.setBrakes(false);

        // Set the magazine to index
        magazineSubsystem.setDefaultCommand(indexer);
    }

    @Override
    public void robotPeriodic() {
        shooterSubsystem.diagnostic();
        CommandScheduler.getInstance().run();
        if (RobotMap.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Controller Temp", swerveDriveSubsystem.getDriveTemperature());
            SmartDashboard.putNumber("Rotation Controller Temp", swerveDriveSubsystem.getRotationTemperature());
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
            swerveDriveSubsystem.setBrakes(false);
            brakesTimerCompleted = true;
        }
    }

    @Override
    public void autonomousInit() {
        Blackbox.getInstance().startLog();
        m_autonomousCommand = autoChooser.getSelected();
        m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
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
