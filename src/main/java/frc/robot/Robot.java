package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import edu.wpi.first.math.geometry.Translation2d;

public class Robot extends TimedRobot {
    // Create hardware objects
    private Gyro gyro = null;
    private SwerveDrive swerveDrive = null;
    private ClimbMotorSubsystem climbMotor = null;
    private ClimbArmSubsystem climbArm = null;
    private HoodSubsystem hood = null;
    private ShooterSubsystem shooter = null;
    private DigitalInput canivoreSwitch = new DigitalInput(6);
    private Limelight limelight = null;

    // Create objects to run auto and teleop code
    public Auto auto = null;
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
    private String autoSelected;
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    private IntakeSubsystem intake;
    private MagazineSubsystem magazine;
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
        swerveDrive = new SwerveDrive(gyro, initialPosition, bus);
        climbMotor = new ClimbMotorSubsystem();
        climbArm = new ClimbArmSubsystem();
        hood = new HoodSubsystem();
        shooter = new ShooterSubsystem(bus);
        intake = new IntakeSubsystem();
        magazine = new MagazineSubsystem();
        indexer = new IndexCommand(magazine);
        limelight = new Limelight();

        // Controller objects
        teleop = new Teleop(swerveDrive, climbMotor, climbArm, intake, hood, magazine, shooter, limelight, gyro);
        auto = new Auto(swerveDrive, intake, magazine, shooter, climbArm, hood);

        // Auto picker
        autoChooser.setDefaultOption("3 Ball Auto (Q2)", "3 Ball Auto (Q2)");
        autoChooser.addOption("3 Ball Auto (RED)(Q2)", "3 Ball Auto (RED)(Q2)");
        autoChooser.addOption("High 2 Ball Auto (Q1)", "High 2 Ball Auto (Q1)");
        autoChooser.addOption("High 2 Ball Auto + Starvation (Q1)", "High 2 Ball Auto + Starvation (Q1)");
        autoChooser.addOption("Low 2 Ball Auto (Q1)", "Low 2 Ball Auto (Q1)");
        autoChooser.addOption("Leave tarmac ONLY", "Leave tarmac ONLY");
        autoChooser.addOption("Shoot low ONLY", "Shoot low ONLY");
        autoChooser.addOption("Shoot high (far) ONLY", "Shoot high (far) ONLY");
        autoChooser.addOption("DO NOTHING", "DO NOTHING");

        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotInit() {
        // Create the auto programs in robotInit because it uses a ton of trigonometry,
        // which is computationally expensive
        auto.createPrograms();

        // Put the auto picker on SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Turn off the brakes
        swerveDrive.setBrakes(false);

        // Set the magazine to index
        magazine.setDefaultCommand(indexer);
    }

    @Override
    public void robotPeriodic() {
        shooter.diagnostic();
        CommandScheduler.getInstance().run();
        if (RobotMap.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Controller Temp", swerveDrive.getDriveTemperature());
            SmartDashboard.putNumber("Rotation Controller Temp", swerveDrive.getRotationTemperature());
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
            swerveDrive.setBrakes(false);
            brakesTimerCompleted = true;
        }
    }

    @Override
    public void autonomousInit() {
        Blackbox.getInstance().startLog();

        autoSelected = autoChooser.getSelected();

        int autoID = 0;
        switch (autoSelected) {
            case "3 Ball Auto (Q2)":
                autoID = 0;
                break;

            case "High 2 Ball Auto (Q1)":
                autoID = 1;
                break;

            case "High 2 Ball Auto + Starvation (Q1)":
                autoID = 2;
                break;

            case "Low 2 Ball Auto (Q1)":
                autoID = 3;
                break;

            case "Leave tarmac ONLY":
                autoID = 4;
                break;

            case "Shoot low ONLY":
                autoID = 5;
                break;

            case "Shoot high (far) ONLY":
                autoID = 6;
                break;

            case "3 Ball Auto (RED)(Q2)":
                autoID = 7;
                break;

            case "DO NOTHING":
                autoID = 8;
                break;

            default:
                DriverStation.reportError(autoSelected + " is not an auto program!", false);
        }

        // System.out.println("Running auto: " + autoSelected + " - (ID " + autoID +
        // ")");
        auto.autoInit(autoID);
    }

    @Override
    public void autonomousPeriodic() {
        auto.autoPeriodic();
    }

    @Override
    public void teleopInit() {
        Blackbox.getInstance().startLog();

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
