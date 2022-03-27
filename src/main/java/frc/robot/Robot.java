package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.IndexCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    //Create hardware objects
    private Gyro gyro = null;
    private SwerveDrive swerveDrive = null;
    private ClimbMotorSubsystem climbMotor = null;
    private ClimbArmSubsystem climbArm = null;
    private HoodSubsystem hood = null;
    private ShooterSubsystem shooter = null;
    private UsbCamera[] cameras = null;
    private VideoSink server = null;

    // Create objects to run auto and teleop code
    public Auto auto = null;
    public Teleop teleop = null;

    //Timer for keeping track of when to disable brakes after being disabled so that the robot stops safely
    private Timer brakesTimer = new Timer();
    private boolean brakesTimerCompleted = false;

    //The initial position of the robot relative to the field. This is measured from the left-hand corner of the field closest to the driver, from the driver's perspective
    public Translation2d initialPosition = new Translation2d(0, 0);

    //Auto selector on SmartDashboard
    private String autoSelected;
    private SendableChooser<String> autoChooser = new SendableChooser<>();

    private NetworkTableEntry batteryVolt;
    private IntakeSubsystem intake;
    private MagazineSubsystem magazine;
    private IndexCommand indexer;

    public Robot() {
        //Hardware-based objects
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gyro = new Gyro();
        gyro.initializeNavX();
        swerveDrive = new SwerveDrive(gyro, initialPosition);
        climbMotor = new ClimbMotorSubsystem();
        climbArm = new ClimbArmSubsystem();
        hood = new HoodSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        magazine = new MagazineSubsystem();
        indexer = new IndexCommand(magazine);
        cameras = new UsbCamera[] {
            CameraServer.startAutomaticCapture("Intake Camera", 0),
            //CameraServer.startAutomaticCapture("Alignment Camera", 1)
        };
        server = CameraServer.getServer();

        //Controller objects
        teleop = new Teleop(swerveDrive, climbMotor, climbArm, intake, hood, magazine, shooter, cameras, server);
        auto = new Auto(swerveDrive, intake, magazine, shooter, climbArm, hood);

        //Auto picker
        autoChooser.setDefaultOption("3 Ball Auto (Q2)", "3 Ball Auto (Q2)");
        autoChooser.addOption("4 Ball Auto (Q2)", "4 Ball Auto (Q2)");
        autoChooser.addOption("High 2 Ball Auto (Q1)", "High 2 Ball Auto (Q1)");
        autoChooser.addOption("Low 2 Ball Auto (Q1)", "Low 2 Ball Auto (Q1)");
        autoChooser.addOption("Leave tarmac ONLY", "Leave tarmac ONLY");
        autoChooser.addOption("Shoot low ONLY", "Shoot low ONLY");
        autoChooser.addOption("Shoot high (far) ONLY", "Shoot high (far) ONLY");
        autoChooser.addOption("DO NOTHING", "DO NOTHING");
    }

    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive
        auto.createPrograms();

        //Put the auto picker on SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Turn off the brakes
        swerveDrive.setBrakes(false);

        //Set up cameras
        cameras[0].setFPS(20);
        cameras[0].setResolution(240, 180);
        //cameras[1].setFPS(20);
        //cameras[1].setResolution(240, 180);

        //Show the toggleable camera feed (this IS the intended way of doing this - the camera stream gets overridden by the server for whatever reason)
        Shuffleboard.getTab("Camera Feed").add("Camera Feed", cameras[0]);
        Shuffleboard.getTab("Driver Dashboard").add("Camera Feed", cameras[0]);
        

        Map<String, Object> propertiesBattery = new HashMap<String, Object>();
        propertiesBattery.put("Min Value", 0);
        propertiesBattery.put("Max Value", 100);
        propertiesBattery.put("Threshold", 10);
        propertiesBattery.put("Angle Range", 180);
        propertiesBattery.put("Color", "red");
        propertiesBattery.put("Threshold Color", "green");


        double volt = (RobotController.getBatteryVoltage() - 11) / 2;
        batteryVolt = Shuffleboard.getTab("Dashboard Refresh")
                .add("Battery Gauge", volt)
                .withWidget("Temperature Gauge") // specify the widget here
                .withProperties(propertiesBattery)
                .getEntry();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        magazine.setDefaultCommand(indexer);
        if (RobotMap.ROBOT_INFO) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Motor Temp", swerveDrive.getDriveTemperature());
            SmartDashboard.putNumber("Rotation Motor Temp", swerveDrive.getRotationTemperature());
        }

        double volt = Math.floor(((RobotController.getBatteryVoltage() - 11.75) / 2) * 100);
        if (volt < 0) {
            volt = 0;
        } else if (volt > 100) {
            volt = 100;
        }
        batteryVolt.setDouble(volt);
    }

    @Override
    public void disabledInit() {
        //Cancel all commands
        CommandScheduler.getInstance().cancelAll();
        //Reset and start the brakes timer
        brakesTimerCompleted = false;
        brakesTimer.reset();
        brakesTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (!brakesTimerCompleted && brakesTimer.get() > 2) {
            //Turn off the brakes
            swerveDrive.setBrakes(false);
            brakesTimerCompleted = true;
        }
    }

    @Override
    public void autonomousInit() {
        autoSelected = autoChooser.getSelected();

        int autoID = 0;
        switch (autoSelected) {
            case "3 Ball Auto (Q2)":
                autoID = 0;
                break;

            case "4 Ball Auto (Q2)":
                autoID = 7;
                break;

            case "High 2 Ball Auto (Q1)":
                autoID = 1;
                break;

            case "Low 2 Ball Auto (Q1)":
                autoID = 2;
                break;

            case "Leave tarmac ONLY":
                autoID = 3;
                break;

            case "Shoot low ONLY":
                autoID = 4;
                break;

            case "Shoot high (far) ONLY":
                autoID = 5;
                break;

            case "DO NOTHING":
                autoID = 6;
                break;

            default:
                DriverStation.reportError(autoSelected + " is not an auto program!", false);
        }

        System.out.println("Running auto: " + autoSelected + " - (ID " + autoID + ")");
        auto.autoInit(autoID);
    }

    @Override
    public void autonomousPeriodic() {
        auto.autoPeriodic();
    }

    @Override
    public void teleopInit() {
        teleop.teleopInit();
    }

    @Override
    public void teleopPeriodic() {
        teleop.teleopPeriodic();
    }
}
