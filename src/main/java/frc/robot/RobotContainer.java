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
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.auto.ThreeBallAutoQ2;
import frc.robot.commands.auto.TwoBallAutoQ1High;
import frc.robot.commands.auto.TwoBallAutoQ1HighStarve;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.CameraMode;
import frc.robot.subsystems.ClimbArmSubsystem;
import frc.robot.subsystems.ClimbMotorSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {

    // The initial position of the robot relative to the field. This is measured
    // from the left-hand corner of the field closest to the driver, from the
    // driver's perspective
    public Translation2d initialPosition = new Translation2d(0, 0);

    // Create hardware objects
    private Pigeon pigeon;
    private final OI joysticks = new OI();

    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ClimbMotorSubsystem m_climbMotorSubsystem;
    private final ClimbArmSubsystem m_climbArmSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final MagazineSubsystem m_magazineSubsystem;

    private AutoClimbCommand autoClimbCommand;
    private VisionAlignCommand visionAlignCommand;
    private boolean visionEnabled = true;
    private boolean visionTargeting = false;

    private double aimRotationSpeed = 0.25 * 0.7;

    // Auto selector on SmartDashboard
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final IndexCommand indexer;

    public RobotContainer() {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        pigeon = new Pigeon();
        AutoPaths.CreateAutoPaths();

        m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "canivore");
        m_climbMotorSubsystem = new ClimbMotorSubsystem();
        m_climbArmSubsystem = new ClimbArmSubsystem();
        m_hoodSubsystem = new HoodSubsystem();
        m_shooterSubsystem = new ShooterSubsystem("canivore");
        m_intakeSubsystem = new IntakeSubsystem();
        m_magazineSubsystem = new MagazineSubsystem();
        m_limelightSubsystem = new LimelightSubsystem();

        indexer = new IndexCommand(m_magazineSubsystem);
        // Set the magazine to index
        m_magazineSubsystem.setDefaultCommand(indexer);
        new RunCommand(m_shooterSubsystem::diagnostic).schedule();
        m_swerveDriveSubsystem.setBrakes(false);

        m_swerveDriveSubsystem.setDefaultCommand(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity, joysticks::getYVelocity,
                        joysticks::getRotationVelocity, joysticks::getSpeed, () -> 0.8 * joysticks.getSpeed()));

        //m_shooterSubsystem.setDefaultCommand(new RunCommand(m_shooterSubsystem::shooterHighFar, m_shooterSubsystem));

        // Auto picker
        autoChooser.setDefaultOption("3 Ball Auto (Q2)",
                new ThreeBallAutoQ2(m_swerveDriveSubsystem, m_intakeSubsystem,
                        m_hoodSubsystem, m_magazineSubsystem, m_shooterSubsystem));
        autoChooser.addOption("High 2 Ball Auto (Q1)",
                new TwoBallAutoQ1High(m_swerveDriveSubsystem, m_intakeSubsystem,
                        m_hoodSubsystem, m_magazineSubsystem, m_shooterSubsystem));
        autoChooser.addOption("High 2 Ball Auto + Starvation (Q1)",
                new TwoBallAutoQ1HighStarve(m_swerveDriveSubsystem,
                        m_intakeSubsystem, m_hoodSubsystem, m_magazineSubsystem,
                        m_shooterSubsystem));
        autoChooser.addOption("DO NOTHING", new WaitCommand(0));
        SmartDashboard.putData(autoChooser);
        SmartDashboard.putData(m_magazineSubsystem);
        LiveWindow.disableAllTelemetry();
        visionAlignCommand = new VisionAlignCommand(m_limelightSubsystem, m_swerveDriveSubsystem);
        autoClimbCommand = new AutoClimbCommand(m_climbArmSubsystem, m_climbMotorSubsystem, joysticks.autoClimb,
                joysticks.abortAutoClimb);
        configureBindings();
    }

    private void configureBindings() {

        /*joysticks.abortVisionAlign
                .whenActive(() -> {
                    if (visionTargeting) {
                        visionTargeting = false;
                        m_limelightSubsystem.setCameraMode(CameraMode.Driver);
                    }
                    visionEnabled = !visionEnabled;
                });*/

        joysticks.autoClimb
                .whenActive(autoClimbCommand);

        joysticks.intake
                .whileActiveOnce(
                        new StartEndCommand(
                                m_intakeSubsystem::intakeOn,
                                m_intakeSubsystem::intakeOff,
                                m_intakeSubsystem));

        joysticks.shootLow
                // Raise the hood
                .whenActive(
                        new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem))

                // Lower the climb arm
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))

                // Turn mag once motor is at speed
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                                new RunCommand(
                                        m_magazineSubsystem::magazineOn,
                                        m_magazineSubsystem)))

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
                //.whenActive(new InstantCommand(m_limelightSubsystem::getTargetDistance, m_limelightSubsystem))
                // Lower the hood
                .whenActive(
                        new InstantCommand(m_hoodSubsystem::hoodIn, m_hoodSubsystem))

                // Vision align
                .whenActive(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint).withTimeout(5),
                                new RunCommand(m_magazineSubsystem::magazineOn,
                                        m_magazineSubsystem)))

                // Lower the climb arm
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))
                .whileActiveOnce(new RunCommand(
                        () -> m_shooterSubsystem.shooterHighFar(),
                        m_shooterSubsystem))
                .whenInactive(
                        new InstantCommand(
                                m_magazineSubsystem::magazineOff,
                                m_magazineSubsystem))
                .whenInactive(
                        new InstantCommand(
                                m_shooterSubsystem::shooterOff, 
                                m_shooterSubsystem));
                

        /*joysticks.shootHighFar.and(new Trigger(() -> !visionEnabled))
                .whileActiveOnce(
                        new RunCommand(
                                m_magazineSubsystem::magazineOn,
                                m_magazineSubsystem));*/

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
                                //TODO fix vision align new VisionAlignCommand(m_limelightSubsystem, m_swerveDriveSubsystem),
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint).withTimeout(2),
                                new RunCommand(m_magazineSubsystem::magazineOn,
                                        m_magazineSubsystem)))
                .whenActive(
                        new InstantCommand(m_climbArmSubsystem::armTilt, m_climbArmSubsystem))
                .whileActiveOnce(new RunCommand(
                        m_shooterSubsystem::shooterLaunchpad,
                        m_shooterSubsystem))
                .whenInactive(
                        new RunCommand(
                                m_magazineSubsystem::magazineOff,
                                m_magazineSubsystem))
                .whenInactive(
                        new RunCommand(
                                m_shooterSubsystem::shooterOff,
                                m_shooterSubsystem));
                                
                        

        /*joysticks.shootLaunchpad.and(new Trigger(() -> !visionEnabled))
                .whileActiveOnce(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(m_shooterSubsystem::nearSetpoint),
                                new RunCommand(
                                        m_magazineSubsystem::magazineOn,
                                        m_magazineSubsystem)));*/

        m_magazineSubsystem.getFullMagazineTrigger()
                .whenActive(
                        new RunCommand(
                                m_magazineSubsystem::magazineIndexShort,
                                m_magazineSubsystem)
                                .withTimeout(0.4));

        joysticks.aimLeft.whenHeld(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                joysticks::getYVelocity, () -> -aimRotationSpeed, joysticks::getSpeed));
        joysticks.aimRight.whenHeld(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                        joysticks::getYVelocity, () -> aimRotationSpeed, joysticks::getSpeed));
    }

    public OI getOI() {
        return joysticks;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
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

    public MagazineSubsystem getMagazineSubsystem() {
        return m_magazineSubsystem;
    }

    public SendableChooser<Command> getAutonomousChooser() {
        return autoChooser;
    }
}
