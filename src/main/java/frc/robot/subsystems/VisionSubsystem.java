package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase{

    public final PhotonCamera camera = new PhotonCamera("targetcam");


    private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private SwerveDrivePoseEstimator poseEstimator;
    private DriveSubsystem driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics, 
            driveSubsystem.m_gyro.getRotation2d(), 
            driveSubsystem.getModulePositions(), 
            driveSubsystem.getPose()
        );
    }

}
