package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
class OI {

    private XboxController driveStick = new XboxController(0);
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private boolean toggleFieldOriented;

    public OI() {
        
    }

    //Periodic function to update controller input
    public void checkInputs() {
        xVelocity = driveStick.getX(Hand.kLeft);
        yVelocity = driveStick.getY(Hand.kLeft);
        rotationVelocity = driveStick.getX(Hand.kRight);
        
        //Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < RobotMap.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < RobotMap.JOY_DEAD_ZONE) { rotationVelocity = 0; }

        toggleFieldOriented = driveStick.getXButtonPressed();
    }

    //Getter functions for controls
    public double getXVelocity() {
        return xVelocity;
    }
    public double getYVelocity() {
        return yVelocity;
    }
    public double getRotationVelocity() {
        return rotationVelocity;
    }
    public boolean getToggleFieldOriented() {
        return toggleFieldOriented;
    }
}
