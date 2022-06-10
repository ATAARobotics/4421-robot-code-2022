package frc.robot;
/*
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;*/
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem.CameraMode;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.*;

public class Teleop {
    // Variables for robot classes
    private SwerveDriveSubsystem swerveDrive = null;
    private OI joysticks = null;
    private LimelightSubsystem limelight = null;
    
    //private final ClimbArmSubsystem m_climbArmSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;
    //private final HoodSubsystem m_hoodSubsystem;

  
    public Teleop(SwerveDriveSubsystem swerveDrive, ClimbMotorSubsystem m_climbMotorSubsystem, ClimbArmSubsystem m_climbArmSubsystem, IntakeSubsystem m_intakeSubsystem, HoodSubsystem m_hoodSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem shooter) {
        // Initialize Classes
        this.m_climbMotorSubsystem = m_climbMotorSubsystem;
        //this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_shooterSubsystem = shooter;
        //this.m_hoodSubsystem = m_hoodSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.swerveDrive = swerveDrive;
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
        
        // visionPID.setTolerance(RobotMap.VISION_TARGET_TOLERANCE);
        //Configure the rotation PID to take the shortest route to the setpoint
        // visionPID.enableContinuousInput(-Math.PI, Math.PI);
        swerveDrive.setDefaultCommand(new DriveCommand(swerveDrive, joysticks::getXVelocity, joysticks::getYVelocity, joysticks::getRotationVelocity, joysticks::getSpeed, () -> 0.8*joysticks.getSpeed()));
    }

    public void teleopPeriodic() {
        //Update inputs from the controller
        joysticks.checkInputs();

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

        /*if (visionTargeting) {
            visionRotationVelocity = 0;

            if (visionTarget == -999) {
                //Collect target data from the limelight
                double measurement = limelight.measure();

                if (measurement == -999) {
                    //Vision has aborted itself
                    visionTargeting = false;
                    limelight.setCameraMode(CameraMode.Driver);
                    CommandScheduler.getInstance().schedule(
                        new StartEndCommand(
                            () -> joysticks.rumbleGunnerOn(),
                            () -> joysticks.rumbleGunnerOff()
                        ).withTimeout(2)
                    );
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
                visionRotationVelocity = visionPID.calculate(gyro.getAngle(), visionTarget);
                System.out.println("Vision error: " + (gyro.getAngle() - visionTarget));

                if (visionPID.atSetpoint()) {
                    targetedTicks++;
                    if (targetedTicks >= RobotMap.TARGETED_TICKS) {
                        //WE HAVE ALIGNED WITH THE TARGET
                        visionRotationVelocity = 0;
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
        }*/

        /*if (joysticks.getToggleFieldOriented()) {
            swerveDrive.setFieldOriented(!swerveDrive.getFieldOriented(), 0);
            swerveDrive.resetHeading();
        }*/
    }

}
