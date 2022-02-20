package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.subsystems.*;

public class Auto {

    private SwerveDrive swerveDrive;

    //Create PIDs to control the position of the robot in the x and y direction
    //TODO Decide if these pid controllers even need to exist - code is currently commented out, but should it be?
    private PIDController xController = new PIDController(0.07, 0, 0.001);
    private PIDController yController = new PIDController(0.07, 0, 0.001);

    //Rotation is controlled independently of linear movement, so we use a separate PID system
    private ProfiledPIDController rotationController = new ProfiledPIDController(0.9, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION));

    private Timer timer = new Timer();

    //Stores all the auto programs
    private AutoCommand[][] autoPrograms;

    private IntakeSubsystem m_intakeSubsystem;
    private MagazineSubsystem m_magazineSubsystem;
    private ShooterSubsystem m_shooterSubsystem;

    //The auto program to run
    private int autoSelected;

    //These keep track of the current command in the program, and whether the command that is running just started
    private int commandRunning = 0;
    private boolean newCommand = true;

    //This logs the path of the robot during autonomous (if enabled in RobotMap)
    private DataLogger pathLogger;

    public Auto(SwerveDrive swerveDrive, IntakeSubsystem m_intakeSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem m_shooterSubsystem) {
        this.swerveDrive = swerveDrive;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.m_shooterSubsystem = m_shooterSubsystem;

        //Create a data logging object to log the path
        if (RobotMap.AUTO_PATH_LOGGING_ENABLED) {
            pathLogger = new DataLogger("autoLog");
        }
    }

    public void autoInit(int autoSelected) {
        //Turn on the brakes
        swerveDrive.setBrakes(true);
        
        this.autoSelected = autoSelected;

        commandRunning = 0;
        newCommand = true;

        //Sets the swerve drive to field-oriented
        swerveDrive.setFieldOriented(true, autoPrograms[this.autoSelected][0].getRotationOffset());

        //Resets the position/heading of the robot (just in case we want to run auto more than once without restarting)
        swerveDrive.resetPosition();
        swerveDrive.resetHeading();

        //Reset the data logger (just in case we want to run auto more than once without restarting)
        if (RobotMap.AUTO_PATH_LOGGING_ENABLED) {
            //Set up the logging file
            pathLogger.setupFile();

            //Save waypoints to the path log
            for (AutoCommand autoCommand : autoPrograms[autoSelected]) {
                if (autoCommand.getCommandType() == 0) {
                    List<Translation2d> waypoints = autoCommand.getWaypoints();
                    for (Translation2d waypoint : waypoints) {
                        pathLogger.writeLine(waypoint.getX() + "," + waypoint.getY());
                    }
                }
            }
            pathLogger.writeLine("PATH LOG STARTS HERE");
        }

        //Configure the rotation PID to take the shortest route to the setpoint
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    //Ignore dead code warning
    @SuppressWarnings("unused")
    public void autoPeriodic() {
        //Get the command in the auto program to run, unless auto is finished
        AutoCommand currentCommand = null;
        if (commandRunning < autoPrograms[autoSelected].length) {
            currentCommand = autoPrograms[autoSelected][commandRunning];
        } else {
            //Auto is complete, so close the data logger if we used it
            if (newCommand && RobotMap.AUTO_PATH_LOGGING_ENABLED) {
                pathLogger.close();
            }
        }

        double xVelocity = 0;
        double yVelocity = 0;
        double rotationVelocity = 0;

        if (currentCommand != null) {
            //Each case is a different action, but 0 is always drive
            //For example, 1 could be shooting a ball, 2 could be activating an intake, and so on
            switch (currentCommand.getCommandType()) {
                //DRIVE
                case 0:
                    if (newCommand) {
                        newCommand = false;
                        timer.reset();
                        timer.start();
                    }

                    //Get the state at this time from the trajectory
                    State desiredState = currentCommand.getState(timer.get());
                
                    //Get the current position of the robot
                    Pose2d currentPose = swerveDrive.getPose();

                    //Get the position we want to be at
                    Pose2d desiredPose = desiredState.poseMeters;
            
                    //Get the current angle of the robot
                    double currentAngle = currentPose.getRotation().getRadians();

                    //Get the angle we want to be at
                    double desiredAngle = currentCommand.getTargetAngle();
            
                    //Get the total speed the robot should be travelling (not accounting for deviations)
                    double totalVelocity = desiredState.velocityMetersPerSecond;
                    
                    //Get the velocity in the X and Y direction based on the heading and total speed
                    xVelocity = totalVelocity * desiredPose.getRotation().getCos();
                    yVelocity = totalVelocity * desiredPose.getRotation().getSin();
            
                    //Adjust the speeds using a PID to account for deviations
                    //xVelocity += xController.calculate(currentPose.getX(), desiredPose.getX());
                    //yVelocity += yController.calculate(currentPose.getY(), desiredPose.getY());
            
                    //Get the current rotational velocity from the rotation PID based on the desired angle
                    rotationVelocity = rotationController.calculate(currentAngle, desiredAngle);

                    //Log the current and expected position (don't change this without changing the path viewer utility to read it properly (if you don't know what that is, ask Jacob))
                    if (RobotMap.AUTO_PATH_LOGGING_ENABLED) {
                        pathLogger.writeLine(desiredPose.getX() + "," + desiredPose.getY() + "," + currentPose.getX() + "," + currentPose.getY() + "," + Timer.getFPGATimestamp());
                    }

                    //Check if the robot finished the path
                    if (desiredState == currentCommand.getLastState()) {
                        commandRunning++;
                        newCommand = true;
                    }
                    break;

                //WAIT
                case 1:
                    if (newCommand) {
                        newCommand = false;
                        timer.reset();
                        timer.start();
                    }
                    if (timer.get() > 1.0) {
                        commandRunning++;
                        newCommand = true;
                    }
                    break;

                //EXTEND INTAKE
                case 2:
                    new InstantCommand(m_intakeSubsystem::intakeOn, m_intakeSubsystem);
                    break;

                //RETRACT INTAKE
                case 3:
                    new InstantCommand(m_intakeSubsystem::intakeOff, m_intakeSubsystem);
                    break;

                default:
                    System.err.println("There is no auto command with type " + currentCommand.getCommandType() + "!");
                    break;
            }
        }

        SmartDashboard.putNumber("Expected X Velocity", xVelocity);
        SmartDashboard.putNumber("Expected Y Velocity", yVelocity);
        SmartDashboard.putNumber("Expected Rotation Velocity", rotationVelocity);

        //Drive the robot based on computed velocities
        swerveDrive.periodic(new SwerveCommand(xVelocity, -yVelocity, rotationVelocity, true, swerveDrive.getHeading()));
    }

    /**
     * Initializes the auto programs
     * 
     * AUTO PROGRAMS SHOULD BE CREATED IN THIS FUNCTION
     */
    public void createPrograms() {
        //Get each driving path
        AutoPaths autoPaths = new AutoPaths();

        //Create auto programs
        autoPrograms = new AutoCommand[][] {
            /*
                Example program (fairly short, but you get the idea):

                {
                    //Go from point A to point B
                    autoPaths.getPathName(),

                    //Do something at point B
                    new AutoCommand(1),

                    //Go back from point B to point A
                    autoPaths.getPathName()
                }

                This array should just be an array of these arrays.
                Each of these are an entire auto program, executed from index 0 to the end of the array.
            */

            //Three ball from Q2 (Preloaded, 4, 5) - IN PROGRESS
            {
                //Shoot preloaded ball

                //Travel to ball 5
                autoPaths.getQuadrant2WallBall5(),
                //Intake out
                new AutoCommand(2),
                //Wait
                new AutoCommand(1),
                //Intake in
                new AutoCommand(3),
                //Travel to ball 4
                autoPaths.getBall5Ball4(),
                //Intake out
                new AutoCommand(2),
                //Wait
                new AutoCommand(1),
                //Intake in
                new AutoCommand(3),
                //Travel to Q2 wall
                autoPaths.getBall4Quadrant2Wall()
                //Shoot balls 4 and 5

            },
        };
    }
}
