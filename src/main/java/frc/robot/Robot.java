package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Robot extends TimedRobot {
    //Create hardware objects
    private Gyro gyro = null;
    private SwerveDrive swerveDrive = null;

    // Create objects to run auto and teleop code
    public Auto auto = null;
    public Teleop teleop = null;

    //The initial position of the robot relative to the field. This is measured from the left-hand corner of the field closest to the driver, from the driver's perspective
    public Translation2d initialPosition = new Translation2d(5.3694, 7.748);

    public Robot() {
        //Hardware-based objects
        gyro = new Gyro();
        gyro.initializeNavX();
        swerveDrive = new SwerveDrive(gyro, initialPosition);

        //Controller objects
        auto = new Auto(swerveDrive);
        teleop = new Teleop(swerveDrive);
    }

    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive
        auto.createPrograms();
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

    }

    @Override
    public void disabledInit() {
        
    }

    @Override
    public void disabledPeriodic() {

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
