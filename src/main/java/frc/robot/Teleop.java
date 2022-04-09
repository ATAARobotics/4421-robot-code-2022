package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight.CameraMode;
import frc.robot.subsystems.*;

public class Teleop {
    // Variables for robot classes
    private SwerveDrive swerveDrive = null;
    private OI joysticks = null;
    private Limelight limelight = null;
    
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private boolean visionTargeting = false;
    private ProfiledPIDController visionPID = new ProfiledPIDController(0.05, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION));
    private int targetedTicks = 0;
  
    public Teleop(SwerveDrive swerveDrive, ClimbMotorSubsystem m_climbMotorSubsystem, ClimbArmSubsystem m_climbArmSubsystem, IntakeSubsystem m_intakeSubsystem, HoodSubsystem m_hoodSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem shooter, Limelight limelight) {
        // Initialize Classes
        this.joysticks = new OI();
        this.m_climbMotorSubsystem = m_climbMotorSubsystem;
        this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_shooterSubsystem = shooter;
        this.m_hoodSubsystem = m_hoodSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        configureBindings();
    }

    public void teleopInit() {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

        //Set the shooter to teleop mode, and disable the shooter and intake
        m_shooterSubsystem.teleopMode();
        m_intakeSubsystem.intakeOff();

        m_shooterSubsystem.pidReset(); //TODO stop the pid from going bobbly

        m_shooterSubsystem.setDefaultCommand(new RunCommand(m_shooterSubsystem::shooterHighFar, m_shooterSubsystem));

        //We don't have to do anything here for setting field oriented to true - auto does that for us
        if (!RobotMap.FIELD_ORIENTED) {
            swerveDrive.setFieldOriented(false, 0);
        }

        limelight.setCameraMode(CameraMode.Driver);
        visionPID.setTolerance(RobotMap.VISION_TARGET_TOLERANCE);
    }

    public void teleopPeriodic() {
        //Update inputs from the controller
        joysticks.checkInputs();

        swerveDrive.swervePeriodic(false);
        m_shooterSubsystem.shooterPeriodic();

        if (RobotMap.LASERSHARK_DIAGNOSTICS) {
            m_magazineSubsystem.lasersharkValues();
        }

        if (RobotMap.REPORTING_DIAGNOSTICS) {
            m_climbMotorSubsystem.diagnostic();
            m_shooterSubsystem.diagnostic();

            SmartDashboard.putNumber("Left Joy X", joysticks.getXVelocity());
            SmartDashboard.putNumber("Left Joy Y", joysticks.getYVelocity());
            SmartDashboard.putNumber("Right Joy X", joysticks.getRotationVelocity());
        }

        //Show the driver if the vision is on target
        if (!visionTargeting && limelight.getAngularDistance() > RobotMap.VISION_TARGET_TOLERANCE) {
            SmartDashboard.putBoolean("Target Aligned", false);
        }

        double xVelocity, yVelocity, rotationVelocity;
        if (visionTargeting) {
            if (visionPID.atSetpoint() && limelight.hasTarget()) {
                targetedTicks++;
                if (targetedTicks >= RobotMap.TARGETED_TICKS) {
                    SmartDashboard.putBoolean("Target Aligned", true);
                    visionTargeting = false;
                    limelight.setCameraMode(CameraMode.Driver);
                    targetedTicks = 0;
                }
            } else {
                targetedTicks = 0;
            }
            xVelocity = 0;
            yVelocity = 0;
            rotationVelocity = visionPID.calculate(limelight.getAngularDistance() * 100);
            System.out.println(limelight.getAngularDistance());
        } else {
            xVelocity = joysticks.getXVelocity();
            yVelocity = joysticks.getYVelocity();
            rotationVelocity = joysticks.getRotationVelocity();
        }

        //Run periodic tasks on the swerve drive, setting the velocity and rotation
        swerveDrive.setSwerveDrive(
            xVelocity * RobotMap.MAXIMUM_SPEED,
            yVelocity * RobotMap.MAXIMUM_SPEED,
            rotationVelocity * RobotMap.MAXIMUM_ROTATIONAL_SPEED * 0.70
        );

        if (joysticks.getToggleFieldOriented()) {
            swerveDrive.setFieldOriented(!swerveDrive.getFieldOriented(), 0);
            swerveDrive.resetHeading();
        }
    }

    private void configureBindings() {

        joysticks.cancelShooterRev
            .toggleWhenPressed(
                new StartEndCommand(
                    () -> { CommandScheduler.getInstance().unregisterSubsystem(m_shooterSubsystem); },
                    () -> { CommandScheduler.getInstance().registerSubsystem(m_shooterSubsystem); }
                )
            );

        joysticks.visionAlign
            .whenPressed(new InstantCommand(() -> {
                limelight.setCameraMode(CameraMode.Vision);
                visionTargeting = true;
                visionPID.reset(limelight.getAngularDistance());
            }));

        joysticks.intake
            .whileActiveOnce(
                new StartEndCommand(
                    m_intakeSubsystem::intakeOn,
                    m_intakeSubsystem::intakeOff,
                m_intakeSubsystem)
            );
            
        joysticks.shootLow
            //Raise the hood
            .whenActive(
                new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem)
            )

            //Lower the climb arm
            .whenActive(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            )

            //Turn mag once motor is at speed
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Turn on the shooter (automatically turns off when released)
            .whileActiveOnce(
                new RunCommand(
                    m_shooterSubsystem::shooterLow,
                m_shooterSubsystem)
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

        joysticks.shootHighFar
            //Lower the hood
            .whenActive(
                new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem)
            )

            //Turn mag once motor is at speed
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Lower the climb arm
            .whenActive(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            );

        joysticks.shootLaunchpad
            //Lower the hood
            .whenActive(
                new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem)
            )

            //Lower the climb arm
            .whenActive(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            )

            //Turn mag once motor is at speed
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            )

            //Turn on the shooter (automatically turns off when released)
            .whileActiveOnce(
                new RunCommand(
                    m_shooterSubsystem::shooterLaunchpad,
                m_shooterSubsystem));
        
        /*m_magazineSubsystem.getFullMagazineTrigger()
            .whenActive(
                new RunCommand(
                    m_magazineSubsystem::magazineTinyOn,
                m_magazineSubsystem)
                .withTimeout(0.6)
            );*/
    }
}
