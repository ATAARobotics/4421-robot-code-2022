package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Index;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import java.util.HashMap;
import java.util.Map;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Robot extends TimedRobot {
    //Create hardware objects
    private Gyro gyro = null;
    private SwerveDrive swerveDrive = null;
    private Climber climber = null;
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

    private NetworkTableEntry batteryVolt;
    private IntakeSubsystem intake;
    private MagazineSubsystem magazine;
    private Index indexer;

    public Robot() {
        //Hardware-based objects
        //NetworkTableInstance inst = NetworkTableInstance.getDefault();
        gyro = new Gyro();
        gyro.initializeNavX();
        swerveDrive = new SwerveDrive(gyro, initialPosition);
        climber = new Climber();
        shooter = new ShooterSubsystem(climber);
        intake = new IntakeSubsystem();
        magazine = new MagazineSubsystem();
        hood = new HoodSubsystem();
        indexer = new Index(magazine);
        /*TODO camera code
        cameras = new UsbCamera[] {
            CameraServer.startAutomaticCapture("Intake Camera", 0),
            CameraServer.startAutomaticCapture("Alignment Camera", 1)
        };
        server = CameraServer.getServer();
        */

        //Controller objects
        auto = new Auto(swerveDrive, intake, magazine, shooter);
        teleop = new Teleop(swerveDrive, climber, intake, hood, magazine, shooter, cameras, server);
    }

    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive
        auto.createPrograms();

        //Turn off the brakes
        swerveDrive.setBrakes(false);

        /* TODO camera code
        //Set up cameras
        cameras[0].setFPS(20);
        cameras[0].setResolution(240, 180);
        cameras[1].setFPS(20);
        cameras[1].setResolution(240, 180);

        //Show the toggleable camera feed (this IS the intended way of doing this - the camera stream gets overridden by the server for whatever reason)
        Shuffleboard.getTab("Camera Feed").add("Camera Feed", cameras[0]);
        */
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
        auto.autoInit(0);
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
