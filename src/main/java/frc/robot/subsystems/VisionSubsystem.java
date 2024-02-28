package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionSubsystem extends SubsystemBase{

    public final PhotonCamera camera = new PhotonCamera("targetcam");


    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = new Transform3d(new Translation3d(0.68, 0.0, 0.27), new Rotation3d(0,0,0));
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);


    public VisionSubsystem(){
    }

    public Pose2d getPose2d(){
        var results = camera.getLatestResult();
        if(results.hasTargets()){
            return photonPoseEstimator.update().get().estimatedPose.toPose2d(); 
        }
        return new Pose2d(0, 0, new Rotation2d(0));
    }

}
