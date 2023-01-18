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
    private AprilTagLimelight aprilTagLimelight = null;
    

    private final Gyro gyro;

    private boolean visionEnabled = false;
    private boolean visionTargeting = false;
    private ProfiledPIDController visionPID = new ProfiledPIDController(0.9, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED / 4, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION / 2));
    private double visionTarget = -999;
    private int targetedTicks = 0;

    private double rotationSpeedMultiplier = 0.25;
  
    public Teleop(SwerveDrive swerveDrive, Limelight limelight, Gyro gyro, AprilTagLimelight aprilTagLimelight) {
        // Initialize Classes
        this.joysticks = new OI();

        this.gyro = gyro;
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.aprilTagLimelight = aprilTagLimelight;
        configureBindings();
    }

    public void teleopInit() {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

        //Set the shooter to teleop mode, and disable the shooter and intake
        

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


        if (RobotMap.LASERSHARK_DIAGNOSTICS) {

        }

        if (RobotMap.REPORTING_DIAGNOSTICS) {


            SmartDashboard.putNumber("Joy X", joysticks.getXVelocity());
            SmartDashboard.putNumber("Joy Y", joysticks.getYVelocity());
            SmartDashboard.putNumber("Rotation", joysticks.getRotationVelocity());
            SmartDashboard.putNumber("Slider", joysticks.getSpeed());
        }

        double xVelocity, yVelocity, rotationVelocity, speed;
        if (visionEnabled) {
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
                rotationVelocity = visionPID.calculate(gyro.getAngle(), visionTarget);
                System.out.println("Vision error: " + (gyro.getAngle() - visionTarget));

                if (visionPID.atSetpoint()) {
                    targetedTicks++;
                    if (targetedTicks >= RobotMap.TARGETED_TICKS) {
                        //WE HAVE ALIGNED WITH THE TARGET
                        rotationVelocity = 0;
                        limelight.setCameraMode(CameraMode.Driver);
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



        /* joysticks.abortVisionAlign
            .whenActive(() -> {
                if (visionTargeting) {
                    visionTargeting = false;
                    limelight.setCameraMode(CameraMode.Driver);
                }

                visionEnabled = !visionEnabled;
            }); */

        joysticks.driveTag.whenActive(() -> {
            if (visionTargeting) {
                visionTargeting = false;
                limelight.setCameraMode(CameraMode.Driver);
            }

            visionEnabled = !visionEnabled;
        });

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
