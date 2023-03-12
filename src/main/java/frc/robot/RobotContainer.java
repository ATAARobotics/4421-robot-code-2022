package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DrivePlaceCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.auto.Square;
import frc.robot.commands.auto.Straight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.CameraMode;
import frc.robot.subsystems.AprilTagLimelight;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class RobotContainer {

    // The initial position of the robot relative to the field. This is measured
    // from the left-hand corner of the field closest to the driver, from the
    // driver's perspective

    public Translation2d initialPosition = new Translation2d(0, 0);

    // Create hardware objects
    private Pigeon pigeon;
    private final OI joysticks = new OI();

    private final SwerveDrive m_swerveDriveSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    // private final LimelightSubsystem m_limelightSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    private VisionAlignCommand visionAlignCommand;
    private boolean visionEnabled = true;
    private boolean visionTargeting = false;

    private double aimRotationSpeed = 0.25 * 0.7;

    public SwerveAutoBuilder autoBuilder;
    
    // Auto Stuff
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();


    public static ProfiledPIDController rotationController = new ProfiledPIDController(0.9, 0, 0.001,
            new TrapezoidProfile.Constraints(Constants.MAXIMUM_ROTATIONAL_SPEED_AUTO,
                    Constants.MAXIMUM_ROTATIONAL_ACCELERATION));

    public RobotContainer() throws IOException {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        pigeon = new Pigeon();

        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        m_swerveDriveSubsystem  = new SwerveParser(swerveJsonDirectory).createSwerveDrive();

        m_climbMotorSubsystem = new ClimbMotorSubsystem();
        m_climbArmSubsystem = new ClimbArmSubsystem();
        m_hoodSubsystem = new HoodSubsystem();
        m_shooterSubsystem = new ShooterSubsystem("canivore");
        m_intakeSubsystem = new IntakeSubsystem();
        // m_autoPaths = new AutoPaths();
        // m_aprilTagLimeLight = new AprilTagLimelight(m_swerveDriveSubsystem.getOdometry());
        

        // this is for PathPlanner, currently not using
        // HashMap<String, Command> eventMap = new HashMap<>();

        // autoBuilder = new SwerveAutoBuilder(
        //         m_swerveDriveSubsystem::getPose,
        //         m_swerveDriveSubsystem::setInitialPose,
        //         new PIDConstants(0.3, 0.0, 0.0),
        //         new PIDConstants(0.1, 0.0, 0.001),
        //         m_swerveDriveSubsystem.setChassisSpeed,
        //         eventMap,
        //         false,
        //         m_swerveDriveSubsystem);

        // Set the magazine to index
        new RunCommand(m_shooterSubsystem::diagnostic).schedule();
        // m_swerveDriveSubsystem.setBrakes(true);

        m_shooterSubsystem.shooterOff();

        // m_swerveDriveSubsystem.setDefaultCommand(
        //         new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                 joysticks::getYVelocity,
        //                 joysticks::getRotationVelocity, () -> 1,
        //                 () -> 1));

        // autoChooser
        // autoChooser.setDefaultOption("Square", new Square(m_swerveDriveSubsystem));
        // autoChooser.addOption("Square", new Square(m_swerveDriveSubsystem));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        LiveWindow.disableAllTelemetry();

        configureBindings();
    }

    public void AutoInit(double rotation) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        rotationController.reset(new TrapezoidProfile.State(rotation, 0.0));

    }

    private void configureBindings() {


        joysticks.shootLow
                // Raise the hood
                .whenActive(
                        new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem))

                // Lower the climb arm
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))

                // Turn on the shooter (automatically turns off when released)
                .whileActiveOnce(
                        new RunCommand(
                                m_shooterSubsystem::shooterLow,
                                m_shooterSubsystem));

        joysticks.climbMotorUp
                .whileActiveOnce(
                        new RunCommand(m_climbMotorSubsystem::up, m_climbMotorSubsystem))

                .whenInactive(m_climbMotorSubsystem::stop, m_climbMotorSubsystem);

        joysticks.climbMotorDown
                .whileActiveOnce(new RunCommand(m_climbMotorSubsystem::down,
                        m_climbMotorSubsystem))

                .whenInactive(m_climbMotorSubsystem::stop, m_climbMotorSubsystem);

        joysticks.climbArm
                .toggleWhenPressed(new StartEndCommand(m_climbArmSubsystem::armTilt,
                        m_climbArmSubsystem::armVertical,
                        m_climbArmSubsystem));

        joysticks.climbSlow
                .whenPressed(() -> m_climbMotorSubsystem.setSlowSpeed())
                .whenReleased(() -> m_climbMotorSubsystem.setNormalSpeed());

        joysticks.shootHighFar
                // .whenActive(new InstantCommand(m_limelightSubsystem::getTargetDistance,
                // m_limelightSubsystem))
                // Lower the hood
                .whenActive(
                        new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem))

                // Vision align
                .whenActive(
                        new SequentialCommandGroup(
                                // visionAlignCommand,
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint)
                                        .withTimeout(3)))

                // Lower the climb arm
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))
                .whileActiveOnce(new RunCommand(
                        () -> m_shooterSubsystem.shooterHighFar(),
                        m_shooterSubsystem))
                .whenInactive(
                        new InstantCommand(
                                m_shooterSubsystem::shooterOff,
                                m_shooterSubsystem));

        /*
         * joysticks.shootHighFar.and(new Trigger(() -> !visionEnabled))
         * .whileActiveOnce(
         * new RunCommand(
         * m_magazineSubsystem::magazineOn,
         * m_magazineSubsystem));
         */

        joysticks.shootLaunchpad
                // Lower the hood
                .whenActive(
                        new InstantCommand(m_hoodSubsystem::hoodOut, m_hoodSubsystem))

                // Lower the climb arm
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))

                // Vision align
                .whenActive(
                        new SequentialCommandGroup(
                                
                                // VisionAlignCommand(m_limelightSubsystem,
                                // m_swerveDriveSubsystem),
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint)
                                        .withTimeout(2)))
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))
                .whileActiveOnce(new RunCommand(
                        m_shooterSubsystem::shooterLaunchpad,
                        m_shooterSubsystem))

                .whenInactive(
                        new RunCommand(
                                m_shooterSubsystem::shooterOff,
                                m_shooterSubsystem));

        /*
         * joysticks.shootLaunchpad.and(new Trigger(() -> !visionEnabled))
         * .whileActiveOnce(
         * new SequentialCommandGroup(
         * new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
         * new RunCommand(
         * m_magazineSubsystem::magazineOn,
         * m_magazineSubsystem)));
         */

        // joysticks.aimLeft.whenHeld(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //         joysticks::getYVelocity, () -> -aimRotationSpeed, joysticks::getSpeed));


        // joysticks.intake.whileTrue(
        //         new DrivePlaceCommand(m_swerveDriveSubsystem, new Pose2d(12.9, 2.7, new Rotation2d(Math.PI))));

    }

    public OI getOI() {
        return joysticks;
    }

    public SwerveDrive  getSwerveDriveSubsystem() {
        return m_swerveDriveSubsystem;
    }

    public ClimbMotorSubsystem getClimbMotorSubsystem() {
        return m_climbMotorSubsystem;
    }

    public ClimbArmSubsystem getClimbArmSubsystem() {
        return m_climbArmSubsystem;
    }

    public HoodSubsystem getHoodSubsystem() {
        return m_hoodSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return m_shooterSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return m_intakeSubsystem;
    }

    public SendableChooser<Command> getAutonomousChooser() {
        return autoChooser;
    }

    public ProfiledPIDController getRotationController() {
        return rotationController;
    }
}
