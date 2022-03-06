package frc.robot;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class Auto {

    private SwerveDrive swerveDrive;

    //Rotation is controlled independently of linear movement, so we use a separate PID system
    private ProfiledPIDController rotationController = new ProfiledPIDController(0.9, 0, 0.001, new TrapezoidProfile.Constraints(RobotMap.MAXIMUM_ROTATIONAL_SPEED, RobotMap.MAXIMUM_ROTATIONAL_ACCELERATION));

    private Timer timer = new Timer();

    //Stores all the auto programs
    private AutoCommand[][] autoPrograms;

    private IntakeSubsystem m_intakeSubsystem;
    private MagazineSubsystem m_magazineSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private ClimbArmSubsystem m_climbArmSubsystem;
    private HoodSubsystem m_hoodSubsystem;

    //The auto program to run
    private int autoSelected;

    //These keep track of the current command in the program, and whether the command that is running just started
    private int commandRunning = 0;
    private AutoCommand currentCommand = null;
    private boolean newCommand = true;

    //This logs the path of the robot during autonomous (if enabled in RobotMap)
    private DataLogger pathLogger;

    private double xVelocity = 0;
    private double yVelocity = 0;
    private double rotationVelocity = 0;

    public Auto(SwerveDrive swerveDrive, IntakeSubsystem m_intakeSubsystem, MagazineSubsystem m_magazineSubsystem, ShooterSubsystem m_shooterSubsystem, ClimbArmSubsystem m_climbArmSubsystem, HoodSubsystem m_hoodSubsystem) {
        this.swerveDrive = swerveDrive;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_magazineSubsystem = m_magazineSubsystem;
        this.m_shooterSubsystem = m_shooterSubsystem;
        this.m_climbArmSubsystem = m_climbArmSubsystem;
        this.m_hoodSubsystem = m_hoodSubsystem;

        //Create a data logging object to log the path
        if (RobotMap.AUTO_PATH_LOGGING_ENABLED) {
            pathLogger = new DataLogger("autoLog");
        }
    }

    public void autoInit(int autoSelected) {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

        //Spin up the shooter, tip the climb arm, and set the shooter to auto mode
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem),
                new InstantCommand(m_shooterSubsystem::autonomousMode, m_shooterSubsystem),
                new InstantCommand(m_shooterSubsystem::shooterLow, m_shooterSubsystem)
            )
        );
        
        this.autoSelected = autoSelected;

        commandRunning = 0;
        newCommand = true;

        //Sets the initial position of the swerve drive and sets to field oriented
        double initialRotation = 0;
        Translation2d initialPosition = new Translation2d(0, 0);
        for (AutoCommand command : autoPrograms[this.autoSelected]) {
            if (command.getCommandType() == 0) {
                initialRotation = command.getRotationOffset();
                initialPosition = command.getState(0).poseMeters.getTranslation();
                break;
            }   
        }
        swerveDrive.setFieldOriented(true, initialRotation);
        swerveDrive.setInitialPose(new Pose2d(initialPosition, new Rotation2d(initialRotation)));

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

        rotationController.reset(new TrapezoidProfile.State(initialRotation, 0.0));
    }

    //Ignore dead code warning
    @SuppressWarnings("unused")
    public void autoPeriodic() {
        //Get the command in the auto program to run, unless auto is finished
        currentCommand = null;
        if (commandRunning < autoPrograms[autoSelected].length) {
            currentCommand = autoPrograms[autoSelected][commandRunning];
        } else {
            //Auto is complete, so close the data logger if we used it
            if (newCommand && RobotMap.AUTO_PATH_LOGGING_ENABLED) {
                pathLogger.close();
            }
        }

        xVelocity = 0;
        yVelocity = 0;
        rotationVelocity = 0;

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
                    if (timer.get() > currentCommand.getArgument()) {
                        commandRunning++;
                        newCommand = true;
                    }
                    break;

                //EXTEND INTAKE
                case 2:
                    CommandScheduler.getInstance().schedule(new InstantCommand(m_intakeSubsystem::intakeOn, m_intakeSubsystem));
                    commandRunning++;
                    break;

                //RETRACT INTAKE
                case 3:
                    CommandScheduler.getInstance().schedule(new InstantCommand(m_intakeSubsystem::intakeOff, m_intakeSubsystem));
                    commandRunning++;
                    break;

                //ACTIVATE SHOOTER
                case 4:
                    CommandScheduler.getInstance().schedule(
                        new SelectCommand(
                            // Maps selector values to commands
                            Map.ofEntries(
                                Map.entry(0, new ParallelCommandGroup(new InstantCommand(m_shooterSubsystem::shooterLow), new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem))),
                                Map.entry(1, new ParallelCommandGroup(new InstantCommand(m_shooterSubsystem::shooterHighClose), new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem))),
                                Map.entry(2, new ParallelCommandGroup(new InstantCommand(m_shooterSubsystem::shooterHighFar), new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem))),
                                Map.entry(-1, new ParallelCommandGroup(
                                    new InstantCommand(() -> DriverStation.reportError("There is no shoot level of " + this.selectShooter((int)currentCommand.getArgument()), false)),
                                    new InstantCommand(m_shooterSubsystem::shooterOff, m_shooterSubsystem)
                                ))),
                            () -> this.selectShooter((int)currentCommand.getArgument()))
                    );
                    commandRunning++;
                    break;

                //DEACTIVATE SHOOTER
                case 5:
                    CommandScheduler.getInstance().schedule(
                        new ParallelCommandGroup(
                            new InstantCommand(m_shooterSubsystem::shooterLow, m_shooterSubsystem),
                            new InstantCommand(m_magazineSubsystem::magazineOff, m_magazineSubsystem)
                        )
                    );
                    commandRunning++;
                    break;

                //RUN MAGAZINE
                case 6:
                    CommandScheduler.getInstance().schedule(
                        new RunCommand(m_magazineSubsystem::magazineOn, m_magazineSubsystem)
                    );
                    commandRunning++;
                    break;

                default:
                    System.err.println("There is no auto command with type " + currentCommand.getCommandType() + "!");
                    break;
            }
        }

        SmartDashboard.putNumber("Expected X Velocity", xVelocity);
        SmartDashboard.putNumber("Expected Y Velocity", yVelocity);
        SmartDashboard.putNumber("Expected Rotation Velocity", rotationVelocity);

        swerveDrive.setDefaultCommand(new RunCommand(() -> swerveDrive.setSwerveDrive(xVelocity,
                -yVelocity, rotationVelocity), swerveDrive));

    }

    private int selectShooter(int shooterSelect) {
        if(shooterSelect >= 0 && shooterSelect <= 3) {
            return shooterSelect;
        }
        else {
            return -1;
        }
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

            //Three ball from Q2 (Preloaded, 4, 5)
            {
                //Intake out
                new AutoCommand(2),
                //Travel to ball 5
                autoPaths.getQuadrant2EdgeBall5(),
                //Wait
                new AutoCommand(1, 0.5),
                //Intake in
                new AutoCommand(3),
                //Activate shooter
                new AutoCommand(4, 2),
                //Go back to shooting position
                autoPaths.getBall5Quadrant2Edge(),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 1.5),
                //Deactivate shooter
                new AutoCommand(5),
                //Intake out
                new AutoCommand(2),
                //Travel to ball 4
                autoPaths.getQuadrant2EdgeBall4(),
                //Wait
                new AutoCommand(1, 0.5),
                //Intake in
                new AutoCommand(3),
                //Activate shooter
                new AutoCommand(4, 2),
                //Travel to shooting position
                autoPaths.getBall4Quadrant2Shoot(),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 1.5),
                //Deactivate shooter
                new AutoCommand(5)
            },

            //Two ball (high) from Q1 (Preloaded, 2) - DONE
            {
                //Intake out
                new AutoCommand(2),
                //Travel to ball 2
                autoPaths.getQuadrant1LeftBall2(),
                //Wait
                new AutoCommand(1, 1),
                //Intake in
                new AutoCommand(3),
                //Activate shooter
                new AutoCommand(4, 2),
                //Move to the line
                autoPaths.getBall2Quadrant1Line(),
                //Wait
                new AutoCommand(1, 0.2),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 4),
                //Deactivate shooter
                new AutoCommand(5)
            },

            //Two ball (low) from Q1 (Preloaded, 2) - IN PROGRESS
            {
                //Intake out
                new AutoCommand(2),
                //Travel to ball 2
                autoPaths.getQuadrant1LeftBall2(),
                //Wait
                new AutoCommand(1, 1),
                //Intake in
                new AutoCommand(3),
                //Activate shooter
                new AutoCommand(4, 0),
                //Move to the line
                autoPaths.getBall2Quadrant1Wall(),
                //Wait
                new AutoCommand(1, 0.2),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 4),
                //Deactivate shooter
                new AutoCommand(5)
            },

            //Leave tarmac ONLY - NOT COMPATIBLE WITH FIELD ORIENTED TELEOP
            {
                //Leave tarmac
                autoPaths.getLeaveTarmac()
            },

            //Shoot low ONLY - NOT COMPATIBLE WITH FIELD ORIENTED TELEOP
            {
                //Activate shooter
                new AutoCommand(4, 0),
                //Wait
                new AutoCommand(1, 2),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 5),
                //Deactivate shooter
                new AutoCommand(5)
            },

            //Shoot high (far) ONLY - NOT COMPATIBLE WITH FIELD ORIENTED TELEOP
            {
                //Activate shooter
                new AutoCommand(4, 2),
                //Wait
                new AutoCommand(1, 2),
                //Activate magazine
                new AutoCommand(6),
                //Wait
                new AutoCommand(1, 5),
                //Deactivate shooter
                new AutoCommand(5)
            },

            //Do literally nothing
            {
                //THESE BRACKETS INTENTIONALLY LEFT BLANK
            }
        };
    }
}
