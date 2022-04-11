package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
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

    private boolean visionEnabled = false;
    private boolean visionTargeting = false;
    private ProfiledPIDController visionPID = new ProfiledPIDController(0.9, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED / 4, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION / 2));
    private double visionTarget = -999;
    private int targetedTicks = 0;

    private double rotationSpeedMultiplier = 0.25;
  
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
        m_intakeSubsystem.intakeOff();

        m_shooterSubsystem.pidReset();

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

            SmartDashboard.putNumber("Joy X", joysticks.getXVelocity());
            SmartDashboard.putNumber("Joy Y", joysticks.getYVelocity());
            SmartDashboard.putNumber("Rotation", joysticks.getRotationVelocity());
            SmartDashboard.putNumber("Slider", joysticks.getSpeed());
        }

        double xVelocity, yVelocity, rotationVelocity, speed;
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

                    System.out.println(visionTarget);

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
                System.out.println("Vision error: " + (gyro.getAngle() - visionTarget));

                if (visionPID.atSetpoint()) {
                    targetedTicks++;
                    if (targetedTicks >= RobotMap.TARGETED_TICKS) {
                        //WE HAVE ALIGNED WITH THE TARGET
                        rotationVelocity = 0;
                        limelight.setCameraMode(CameraMode.Driver);

                        CommandScheduler.getInstance().schedule(
                            new SequentialCommandGroup(
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                                new RunCommand(
                                    m_magazineSubsystem::magazineOn,
                                m_magazineSubsystem).withTimeout(2),
                                new InstantCommand(m_magazineSubsystem::magazineOff),
                                new InstantCommand(() -> {
                                    visionTargeting = false;
                                    System.out.println("Driver control");
                                })
                            )
                        );
                    }
                } else {
                    targetedTicks = 0;
                }
            }
        } else {
            speed = joysticks.getSpeed();
            xVelocity = joysticks.getXVelocity()*speed;
            yVelocity = joysticks.getYVelocity()*speed;
            rotationVelocity = joysticks.getRotationVelocity()*speed* 0.80;
            
        }

        //Run periodic tasks on the swerve drive, setting the velocity and rotation
        swerveDrive.setSwerveDrive(
            xVelocity * RobotMap.MAXIMUM_SPEED,
            yVelocity * RobotMap.MAXIMUM_SPEED,
            rotationVelocity * RobotMap.MAXIMUM_ROTATIONAL_SPEED 
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
                    m_shooterSubsystem::shooterLow,
                m_shooterSubsystem)
            );

        joysticks.climbMotorUp
            .whileActiveOnce(new RunCommand(m_climbMotorSubsystem::climberUp, m_climbMotorSubsystem))

            .whenInactive(m_climbMotorSubsystem::climberStop, m_climbMotorSubsystem);

        joysticks.climbMotorDown
            .whileActiveOnce(new RunCommand(m_climbMotorSubsystem::climberDown, m_climbMotorSubsystem))

            .whenInactive(m_climbMotorSubsystem::climberStop, m_climbMotorSubsystem);

        joysticks.climbArm
            .toggleWhenPressed(new StartEndCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem::armVertical, m_climbArmSubsystem));
        
        joysticks.climbSlow
            .whenPressed(() -> m_climbMotorSubsystem.climberSlowSpeed())
            .whenReleased(() -> m_climbMotorSubsystem.climberNormalSpeed());

        joysticks.climbFast
            .whenPressed(() -> m_climbMotorSubsystem.climberMaxSpeed())
            .whenReleased(() -> m_climbMotorSubsystem.climberNormalSpeed());

        joysticks.shootHighFar
            //Lower the hood
            .whenActive(
                new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem)
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
                new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem)
            )

            //Lower the climb arm
            .whenActive(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem)
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
            .whenActive(
                new RunCommand(
                    m_shooterSubsystem::shooterLaunchpad,
                m_shooterSubsystem));

        joysticks.shootLaunchpad.and(new Trigger(() -> !visionEnabled))
            .whileActiveOnce(
                new SequentialCommandGroup(
                    new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                    new RunCommand(
                        m_magazineSubsystem::magazineOn,
                    m_magazineSubsystem)
                )
            );

        m_magazineSubsystem.getFullMagazineTrigger()
            .whenActive(
                new RunCommand(
                    m_magazineSubsystem::magazineIndexShort,
                m_magazineSubsystem)
                .withTimeout(0.4)
            );

        joysticks.aimLeft.whileHeld(new RunCommand(() -> swerveDrive.setSwerveDrive(
            joysticks.getXVelocity() * RobotMap.MAXIMUM_SPEED, 
            joysticks.getYVelocity() * RobotMap.MAXIMUM_SPEED, 
            -rotationSpeedMultiplier * RobotMap.MAXIMUM_ROTATIONAL_SPEED * 0.70
        ), swerveDrive));
        
        joysticks.aimRight.whileHeld(new RunCommand(() -> swerveDrive.setSwerveDrive(
            joysticks.getXVelocity() * RobotMap.MAXIMUM_SPEED, 
            joysticks.getYVelocity() * RobotMap.MAXIMUM_SPEED, 
            rotationSpeedMultiplier * RobotMap.MAXIMUM_ROTATIONAL_SPEED * 0.70)
        ));
    }

    public boolean shouldClimb() {
        return DriverStation.getMatchTime() <= RobotMap.CLIMB_TIME;
    }
}
