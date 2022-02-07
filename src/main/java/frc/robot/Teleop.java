package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class Teleop {
    // Variables for robot classes
    private SwerveDrive swerveDrive = null;
    private OI joysticks = null;
    private Climber climber = null;
    
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;

    private UsbCamera[] cameras = null;
    private VideoSink cameraServer = null;
    private int cameraActive = 0;
    private XboxController m_driverController;
    private XboxController m_gunnerController;

    public Teleop(SwerveDrive swerveDrive, Climber climber, IntakeSubsystem m_intakeSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem m_shooterSubsystem, UsbCamera[] cameras, VideoSink cameraServer) {
        // Initialize Classes
        this.joysticks = new OI();
        this.climber = climber;
        this.m_shooterSubsystem = m_shooterSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.swerveDrive = swerveDrive;
        this.cameras = cameras;
        this.cameraServer = cameraServer;
        m_driverController = new XboxController(0);
        m_gunnerController = new XboxController(1);
        m_magazineSubsystem.setDefaultCommand(new InstantCommand(m_magazineSubsystem::magazineOff));
        configureButtonBindings();
    }

    public void teleopInit() {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

        if (!RobotMap.FIELD_ORIENTED) {
            swerveDrive.setFieldOriented(false, 0);
        }
    }

    public void teleopPeriodic() {
        //Update inputs from the controller
        joysticks.checkInputs();

        if (RobotMap.DETAILED_JOYSTICK_INFORMATION) {
            SmartDashboard.putNumber("Left Joy X", joysticks.getXVelocity());
            SmartDashboard.putNumber("Left Joy Y", joysticks.getYVelocity());
            SmartDashboard.putNumber("Right Joy X", joysticks.getRotationVelocity());
        }

        //Run periodic tasks on the swerve drive, setting the velocity and rotation
        swerveDrive.periodic(new SwerveCommand(joysticks.getXVelocity() * RobotMap.MAXIMUM_SPEED, joysticks.getYVelocity() * RobotMap.MAXIMUM_SPEED, joysticks.getRotationVelocity() * RobotMap.MAXIMUM_ROTATIONAL_SPEED, swerveDrive.getFieldOriented(), swerveDrive.getHeading()));
    
        if (joysticks.getToggleFieldOriented()) {
            swerveDrive.setFieldOriented(!swerveDrive.getFieldOriented(), 0);
            swerveDrive.resetHeading();
        }

        if (joysticks.getElevatorSpeedDecreased()) {
            climber.decreaseSpeed();
        }
        else if (joysticks.getElevatorSpeedIncreased()) {
            climber.increaseSpeed();
        }

        climber.climberDirectionEnable(joysticks.getElevatorDirection());

        if (joysticks.getToggleClimbArm()) {
            climber.toggleArm();
        }



        /* TODO camera code
        if (joysticks.getToggleCamera()) {
            if (cameraActive == 0) {
                cameraActive = 1;
            } else {
                cameraActive = 0;
            }
            cameraServer.setSource(cameras[cameraActive]);
        }*/
    }

    private void configureButtonBindings() {
        new JoystickButton(m_gunnerController, Button.kLeftBumper.value)
            .toggleWhenPressed(new InstantCommand(m_magazineSubsystem::magazineOn, m_magazineSubsystem))
            .toggleWhenPressed(new StartEndCommand(m_intakeSubsystem::intakeOn, m_intakeSubsystem::intakeOff, m_intakeSubsystem));
      new JoystickButton(m_gunnerController, Button.kA.value)
      .toggleWhenPressed(new InstantCommand(m_magazineSubsystem::magazineOn, m_magazineSubsystem))
      .toggleWhenPressed(new StartEndCommand(m_shooterSubsystem::shooterPercentage, m_shooterSubsystem::shooterOff, m_shooterSubsystem));
    }
}
