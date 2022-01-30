package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Translation2d;

public class Robot extends TimedRobot {
    //Create hardware objects
    private Gyro gyro = null;
    private SwerveDrive swerveDrive = null;
    private UsbCamera[] cameras = null;
    private VideoSink server = null;

    // Create objects to run auto and teleop code
    public Auto auto = null;
    public Teleop teleop = null;

    //Timer for keeping track of when to disable brakes after being disabled so that the robot stops safely
    private Timer brakesTimer = new Timer();
    private boolean brakesTimerCompleted = false;

    //The initial position of the robot relative to the field. This is measured from the left-hand corner of the field closest to the driver, from the driver's perspective
    public Translation2d initialPosition = new Translation2d(5.3694, 7.748);

    public Robot() {
        //Hardware-based objects
        gyro = new Gyro();
        gyro.initializeNavX();
        swerveDrive = new SwerveDrive(gyro, initialPosition);
        /*TODO camera code
        cameras = new UsbCamera[] {
            CameraServer.startAutomaticCapture("Intake Camera", 0),
            CameraServer.startAutomaticCapture("Alignment Camera", 1)
        };
        server = CameraServer.getServer();
        */

        //Controller objects
        auto = new Auto(swerveDrive);
        teleop = new Teleop(swerveDrive, cameras, server);
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
    }

    /**
    * This function is called every robot packet, no matter the mode. Use
    * this for items like diagnostics that you want ran during disabled,
    * autonomous, teleoperated and test.
    *
    * This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    *
    */
    @Override
    public void robotPeriodic() {
        if (RobotMap.ROBOT_INFO) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Motor Temp", swerveDrive.getDriveTemperature());
            SmartDashboard.putNumber("Rotation Motor Temp", swerveDrive.getRotationTemperature());
        }
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
