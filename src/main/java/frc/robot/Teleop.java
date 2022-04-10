package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final Gyro gyro;

    private boolean visionEnabled = true;
    private boolean visionTargeting = false;
    private ProfiledPIDController visionPID = new ProfiledPIDController(0.9, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION));
    private double visionTarget = -999;
    private int targetedTicks = 0;
    private Runnable visionMagazine;
  
    public Teleop(SwerveDrive swerveDrive, ClimbMotorSubsystem m_climbMotorSubsystem, ClimbArmSubsystem m_climbArmSubsystem, IntakeSubsystem m_intakeSubsystem, HoodSubsystem m_hoodSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem shooter, Limelight limelight, Gyro gyro) {
        // Initialize Classes
        this.joysticks = new OI();
        this.m_climbMotorSubsystem = m_climbMotorSubsystem;
        this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_shooterSubsystem = shooter;
        this.m_hoodSubsystem = m_hoodSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.gyro = gyro;
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
        visionTargeting = false;
        visionPID.setTolerance(RobotMap.VISION_TARGET_TOLERANCE);
        //Configure the rotation PID to take the shortest route to the setpoint
        visionPID.enableContinuousInput(-Math.PI, Math.PI);
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

        double xVelocity, yVelocity, rotationVelocity;
        if (visionTargeting) {
            xVelocity = 0;
            yVelocity = 0;
            rotationVelocity = 0;

            if (visionTarget == -999) {
                //Collect target data from the limelight
                double measurement = limelight.measure();

                if (measurement == -999) {
                    //Vision has aborted itself
                    visionTargeting = false;
                    limelight.setCameraMode(CameraMode.Driver);
                    joysticks.rumbleGunner();
                } else if (measurement != 999) {
                    //Vision has picked a target and is ready to align
                    visionTarget = gyro.getAngle() + measurement;

                    //Offset by Pi to find values in the wrong half of the circle
                    visionTarget += Math.PI;

                    //Wrap angle at 2*Pi
                    visionTarget %= 2.0 * Math.PI;

                    //Ensure the value is not negative
                    if (visionTarget < 0) {
                        visionTarget += 2.0 * Math.PI;
                    }

                    //Undo the offset
                    visionTarget -= Math.PI;
                }
            }

            if (visionTarget != -999) {
                rotationVelocity = visionPID.calculate(gyro.getAngle(), visionTarget);

                if (visionPID.atSetpoint()) {
                    targetedTicks++;
                    if (targetedTicks >= RobotMap.TARGETED_TICKS) {
                        //WE HAVE ALIGNED WITH THE TARGET
                        rotationVelocity = 0;
                        limelight.setCameraMode(CameraMode.Driver);
                        targetedTicks = 0;

                        CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                                new RunCommand(
                                    visionMagazine,
                                m_magazineSubsystem)
                            )
                        );
                    }
                } else {
                    targetedTicks = 0;
                }
            }
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

        joysticks.abortVisionAlign
            .whenActive(() -> {
                if (visionTargeting) {
                    visionTargeting = false;
                    limelight.setCameraMode(CameraMode.Driver);
                }

                visionEnabled = !visionEnabled;
            });

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

            //Store the speed for the magazine
            .whenActive(
                () -> visionMagazine = m_magazineSubsystem::magazineOn 
            )

            //Vision align
            .whenActive(() -> {
                if (visionEnabled) {
                    if (!visionTargeting) {
                        limelight.setCameraMode(CameraMode.Vision);
                        limelight.resetTarget();
                        visionTarget = -999;
                        targetedTicks = 0;
                        visionTargeting = true;
                    }
                }
            })

            //Lower the climb arm
            .whenActive(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
            );

        joysticks.shootHighFar.and(new Trigger(() -> !visionEnabled))
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
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

            //Store the speed for the magazine
            .whenActive(
                () -> visionMagazine = m_magazineSubsystem::launchpadmagazineOn 
            )

            //Vision align
            .whenActive(() -> {
                if (visionEnabled) {
                    if (!visionTargeting) {
                        limelight.setCameraMode(CameraMode.Vision);
                        limelight.resetTarget();
                        visionTarget = -999;
                        targetedTicks = 0;
                        visionTargeting = true;
                    }
                }
            })

            //Turn on the shooter (automatically turns off when released)
            .whileActiveOnce(
                new RunCommand(
                    m_shooterSubsystem::shooterLaunchpad,
                m_shooterSubsystem));

        joysticks.shootLaunchpad.and(new Trigger(() -> !visionEnabled))
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::launchpadmagazineOn,
                    m_magazineSubsystem)
                )
            );
        
        /*m_magazineSubsystem.getFullMagazineTrigger()
            .whenActive(
                new RunCommand(
                    m_magazineSubsystem::magazineTinyOn,
                m_magazineSubsystem)
                .withTimeout(0.6)
            );*/
    }
}
