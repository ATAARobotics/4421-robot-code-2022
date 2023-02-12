package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTag {

    public int id;
    public Pose3d aprilTagPose;
    
    AprilTag(int id, double x, double y, double z, double W, double X, double Y, double Z) {
        this.id = id;
        this.aprilTagPose = new Pose3d(x, y, z, new Rotation3d(new Quaternion(W, X, Y, Z)));
    };
}
