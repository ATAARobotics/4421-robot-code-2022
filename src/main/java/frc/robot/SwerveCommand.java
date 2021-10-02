package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveCommand {

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private double[] velocities;
    private double[] angles;

    public SwerveCommand(double xVelocity, double yVelocity, double rotationVelocity, boolean fieldOriented, double gyroAngle) {

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotationVelocity = rotationVelocity;

        if (fieldOriented) {
            double originalX = this.xVelocity;
            double originalY = this.yVelocity;

            this.xVelocity = originalX * Math.cos(gyroAngle) - originalY * Math.sin(gyroAngle);
            this.yVelocity = originalY * Math.cos(gyroAngle) + originalX * Math.sin(gyroAngle);
        }

        //Get the wheelbase and track width from RobotMap. These are important because a long rectangular robot turns differently than a square robot
        double wheelbase = RobotMap.WHEELBASE;
        double trackWidth = RobotMap.TRACK_WIDTH;

        SmartDashboard.putNumber("X Velocity", this.xVelocity);
        SmartDashboard.putNumber("Y Velocity", this.yVelocity);
        SmartDashboard.putNumber("Rotation Velocity", this.rotationVelocity);

        //Calculate wheel velocities and angles
        double a,b,c,d;
        
        a = this.xVelocity - this.rotationVelocity * wheelbase / 2;
        b = this.xVelocity + this.rotationVelocity * wheelbase / 2;
        c = this.yVelocity - this.rotationVelocity * trackWidth / 2;
        d = this.yVelocity + this.rotationVelocity * trackWidth / 2;

        velocities = new double[]{
            Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2)),
            Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2)),
            Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2)),
            Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2))
        };
        angles = new double[]{
            //Math.atan2(y, x) computes the angle to a given point from the x axis
            Math.atan2(b, d),
            Math.atan2(b, c),
            Math.atan2(a, d),
            Math.atan2(a, c)
        };

        /*
        //Get the maximum wheel speed
        double maxSpeed = Math.max(Math.max(Math.abs(velocities[0]), Math.abs(velocities[1])), Math.max(Math.abs(velocities[2]), Math.abs(velocities[3])));

        //If any speed is larger than the maximum speed, all speeds need to be reduced to keep the ratio between the speeds correct
        if (maxSpeed > RobotMap.MAXIMUM_SPEED) {
            for (int i = 0; i < velocities.length; i++) {
                //Bring the speeds down to a scale from 0 to 1, 1 being the highest speed
                velocities[i] /= maxSpeed;
                //Multiply by the maximum speed - this will make the highest speed the maximum speed, and make all the other values scale properly
                velocities[i] *= RobotMap.MAXIMUM_SPEED;
            }
        }
        */
    }

    /**
     * Gets the velocity of a specific module in meters/second
     * @param moduleId The ID of the module to get
     */
    public double getModuleVelocity(int moduleId) {
        return velocities[moduleId];
    }

    /**
     * Gets the angle that the module should be set to
     * @param moduleId The ID of the module to get
     */
    public double getModuleAngle(int moduleId) {
        return angles[moduleId];
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getRotationVelocity() {
        return rotationVelocity;
    }
}
