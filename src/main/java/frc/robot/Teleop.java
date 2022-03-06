package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbTwoCommand;
import frc.robot.commands.EjectBallCommand;
import frc.robot.subsystems.*;

public class Teleop {
    // Variables for robot classes
    private SwerveDrive swerveDrive = null;
    private OI joysticks = null;
    
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;
    private final HoodSubsystem m_hoodSubsystem;

    //TODO: Add with camera code
    //private UsbCamera[] cameras = null;
    //private VideoSink cameraServer = null;
    //private int cameraActive = 0;

    public Teleop(SwerveDrive swerveDrive, ClimbMotorSubsystem m_climbMotorSubsystem, ClimbArmSubsystem m_climbArmSubsystem, IntakeSubsystem m_intakeSubsystem, HoodSubsystem m_hoodSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem shooter, UsbCamera[] cameras, VideoSink cameraServer) {
        // Initialize Classes
        this.joysticks = new OI();
        this.m_climbMotorSubsystem = m_climbMotorSubsystem;
        this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_shooterSubsystem = shooter;
        this.m_hoodSubsystem = m_hoodSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.swerveDrive = swerveDrive;
        //TODO: Add back with camera code
        //this.cameras = cameras;
        //this.cameraServer = cameraServer;

        configureBindings();
    }

    public void teleopInit() {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

        //Set the shooter to teleop mode
        CommandScheduler.getInstance().schedule(
            new InstantCommand(m_shooterSubsystem::teleopMode, m_shooterSubsystem)
        );

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
        swerveDrive.setSwerveDrive(
            joysticks.getXVelocity() * RobotMap.MAXIMUM_SPEED, 
            joysticks.getYVelocity() * RobotMap.MAXIMUM_SPEED, 
            joysticks.getRotationVelocity() * RobotMap.MAXIMUM_ROTATIONAL_SPEED
        );
    
        if (joysticks.getToggleFieldOriented()) {
            swerveDrive.setFieldOriented(!swerveDrive.getFieldOriented(), 0);
            swerveDrive.resetHeading();
        }

        /* TODO camera code
        if (joysticks.getSwitchCameras()) {
            if (cameraActive == 0) {
                cameraActive = 1;
            } else {
                cameraActive = 0;
            }
            cameraServer.setSource(cameras[cameraActive]);
        }*/
    }

    private void configureBindings() {

        joysticks.reverseBalls
            .whileHeld(new EjectBallCommand(m_shooterSubsystem, m_magazineSubsystem, m_intakeSubsystem));        

        joysticks.intakeUpOnly
            .whenPressed(new InstantCommand(m_intakeSubsystem::intakeOff, m_intakeSubsystem));

        joysticks.intake
            .toggleWhenPressed(
                new StartEndCommand(
                    m_intakeSubsystem::intakeOn,
                    m_intakeSubsystem::intakeOff,
                m_intakeSubsystem)
            );

        joysticks.shootLow
            //Lower the hood
            .toggleWhenPressed(
                new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem)
            )

            //Lower the climb arm
            .toggleWhenPressed(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            )

            //Turn on the shooter (automatically turns off when released)
            .whenHeld(
                new StartEndCommand(
                    m_shooterSubsystem::shooterLow,
                    m_shooterSubsystem::shooterOff,
                m_shooterSubsystem)
            )

            //Turn on the magazine after 1 second
            .whenHeld(
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Turn off the magazine
            .whenReleased(
                new InstantCommand(
                    m_magazineSubsystem::magazineOff,
                m_magazineSubsystem)
            );

        joysticks.climbMotorUp
            .whileHeld(new StartEndCommand(
                m_climbMotorSubsystem::climberUp,
                m_climbMotorSubsystem::climberStop,
                m_climbMotorSubsystem
            )
        );

        joysticks.climbMotorDown
            .whileHeld(new StartEndCommand(
                m_climbMotorSubsystem::climberDown, 
                m_climbMotorSubsystem::climberStop,
                m_climbMotorSubsystem
            )
        );

        joysticks.climbArm
            .toggleWhenPressed(new StartEndCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem::armVertical, m_climbArmSubsystem));
        
        joysticks.climbSlow
            .whenPressed(new InstantCommand(m_climbMotorSubsystem::climberSlowSpeed, m_climbMotorSubsystem))
            .whenReleased(new InstantCommand(m_climbMotorSubsystem::climberNormalSpeed, m_climbMotorSubsystem));

        joysticks.climbFast
            .whenPressed(new InstantCommand(m_climbMotorSubsystem::climberMaxSpeed, m_climbMotorSubsystem))
            .whenReleased(new InstantCommand(m_climbMotorSubsystem::climberNormalSpeed, m_climbMotorSubsystem));

        /* joysticks.autoClimbUp
            .whenPressed(new ClimbTwoCommand(this.m_climbArmSubsystem, this.m_climbMotorSubsystem));
        
        joysticks.autoClimbTwo
            .whenPressed(new ClimbTwoCommand(this.m_climbArmSubsystem, this.m_climbMotorSubsystem));
        
        joysticks.autoClimbSwing
            .whenPressed(new ClimbTwoCommand(this.m_climbArmSubsystem, this.m_climbMotorSubsystem)); */

        

        /*joysticks.magazine
            .toggleWhenPressed(new RunCommand(m_magazineSubsystem::magazineOn, m_magazineSubsystem));
            //Turn on the magazine after 1 second
            .whenHeld(
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )

            //Turn off the magazine
            .whenReleased(
                new InstantCommand(
                    m_magazineSubsystem::magazineOff,
                m_magazineSubsystem)
            );*/

        joysticks.shootHighClose
            //Raise the hood
            .whenPressed(
                new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem)
            )

            //Lower the climb arm
            .toggleWhenPressed(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            )

            //Turn on the shooter (automatically turns off when released)
            .whenHeld(
                new StartEndCommand(
                    m_shooterSubsystem::shooterHighClose,
                    m_shooterSubsystem::shooterOff,
                m_shooterSubsystem))

            //Turn on the magazine after 1 second
            .whenHeld(
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Turn off the magazine
            .whenReleased(
                new InstantCommand(
                    m_magazineSubsystem::magazineOff,
                m_magazineSubsystem)
            );

        joysticks.shootHighFar
            //Lower the hood
            .whenPressed(
                new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem)
            )

            //Lower the climb arm
            .toggleWhenPressed(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            )

            //Turn on the shooter (automatically turns off when released)
            .whenHeld(
                new StartEndCommand(
                    m_shooterSubsystem::shooterHighFar,
                    m_shooterSubsystem::shooterOff,
                m_shooterSubsystem))

            //Turn on the magazine after 1 second
            .whenHeld(
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Turn off the magazine
            .whenReleased(
                new InstantCommand(
                    m_magazineSubsystem::magazineOff,
                m_magazineSubsystem)
            );
        
        m_magazineSubsystem.getFullMagazineTrigger()
            .whenActive(
                new ScheduleCommand(
                    new RunCommand(
                        m_magazineSubsystem::magazineTinyOn,
                    m_magazineSubsystem)
                    .withTimeout(0.4)
                )
            );
    }
}
