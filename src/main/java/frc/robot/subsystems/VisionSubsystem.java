package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase{

    public final PhotonCamera camera = new PhotonCamera("targetcam");


    private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private SwerveDrivePoseEstimator poseEstimator;
    private DriveSubsystem driveSubsystem;



    public Vector<N3> createStateStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }

    public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
        return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
    }

    public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
        return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
    }
    

    public VisionSubsystem(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics, 
            driveSubsystem.m_gyro.getRotation2d(), 
            driveSubsystem.getModulePositions(), 
            driveSubsystem.getPose(),
            createStateStdDevs(.1, .1, 10),
            createVisionMeasurementStdDevs(5, 5, 500)
        );
    }



}
