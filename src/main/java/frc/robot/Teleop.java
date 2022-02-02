package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
    // Variables for robot classes
    private SwerveDrive swerveDrive = null;
    private OI joysticks = null;
    private Climber climber = null;
    private Shooter shooter = null;
    private UsbCamera[] cameras = null;
    private VideoSink cameraServer = null;
    private int cameraActive = 0;

    public Teleop(SwerveDrive swerveDrive, Climber climber, Shooter shooter, UsbCamera[] cameras, VideoSink cameraServer) {
        // Initialize Classes
        this.joysticks = new OI();
        this.climber = climber;
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;
        this.cameras = cameras;
        this.cameraServer = cameraServer;
    }

    public void teleopInit() {
        //Turn on the brakes
        swerveDrive.setBrakes(true);

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
        swerveDrive.periodic(new SwerveCommand(joysticks.getXVelocity() * RobotMap.MAXIMUM_SPEED, joysticks.getYVelocity() * RobotMap.MAXIMUM_SPEED, joysticks.getRotationVelocity() * RobotMap.MAXIMUM_ROTATIONAL_SPEED, swerveDrive.getFieldOriented(), swerveDrive.getHeading()));
    
        if (joysticks.getToggleFieldOriented()) {
            swerveDrive.setFieldOriented(!swerveDrive.getFieldOriented(), 0);
            swerveDrive.resetHeading();
        }

        if (joysticks.getElevatorSpeedDecreased()) {
            climber.decreaseSpeed();
        }
        else if (joysticks.getElevatorSpeedIncreased()) {
            climber.increaseSpeed();
        }

        climber.climberDirectionEnable(joysticks.getElevatorDirection());
        
        if (joysticks.getToggleIntake()) {
            shooter.toggleIntake();
        }

        if (joysticks.getToggleClimbArm()) {
            climber.toggleArm();
        }


        /* TODO camera code
        if (joysticks.getToggleCamera()) {
            if (cameraActive == 0) {
                cameraActive = 1;
            } else {
                cameraActive = 0;
            }
            cameraServer.setSource(cameras[cameraActive]);
        }*/
    }
}
