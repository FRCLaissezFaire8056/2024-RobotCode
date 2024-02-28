package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

public class VisionSubsystem extends SubsystemBase{

    private final double kTurningP = VisionConstants.kTurningP;
    private final double kTurningI = VisionConstants.kTurningI;
    private final double kTurningD = VisionConstants.kTurningD;

    private final double kDrivingP = VisionConstants.kDrivingP;
    private final double kDrivingI = VisionConstants.kDrivingI;
    private final double kDrivingD = VisionConstants.kDrivingD;

    private final double kAmpHeight = VisionConstants.kAmpHeight;
    private final double kCamHeight = VisionConstants.kCamHeight;
    private final double kCamAngle = VisionConstants.kCamAngle;

    private double turningSpeed = 0;
    private double drivingSpeed = 0;

    public final PhotonCamera camera = new PhotonCamera("targetcam");
    private final PIDController turningController = new PIDController(kTurningP, kTurningI, kTurningD);
    private final PIDController drivingController = new PIDController(kDrivingP, kDrivingI, kDrivingD);

    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    Transform3d robotToCam = new Transform3d(new Translation3d(0.68, 0.0, 0.27), new Rotation3d(0,0,0));
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);


    public VisionSubsystem(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        camera.setPipelineIndex(0);
    }

    public void caluclate(){
        var result = camera.getLatestResult();
        if(result.hasTargets()){
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                kCamHeight, 
                kAmpHeight, 
                kCamAngle, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            drivingSpeed = drivingController.calculate(range, 1);
            turningSpeed = turningController.calculate(result.getBestTarget().getYaw(), 0);
        }
        else{
            turningSpeed = 0;
            drivingSpeed = 0;
        }
        
        System.out.println("turning " + turningSpeed + "\n" + "driving " + drivingSpeed);

        driveSubsystem.drive(drivingSpeed, 0, turningSpeed, false, false);
    }

    public Pose2d getPose2d(){
        var results = camera.getLatestResult();
        System.out.print(results.getBestTarget());
        if(results.hasTargets()){
            var updt = photonPoseEstimator.update();
            System.out.println(updt);
            System.out.println("-----------------------------------");
            Pose3d currPose3d = updt.get().estimatedPose;
            System.out.println(currPose3d);
            System.out.println("-----------------------------------");
            Pose2d robotPose2d = currPose3d.toPose2d();
            System.out.println(robotPose2d);
            System.out.println("-----------------------------------");

            return robotPose2d;
        }
        return new Pose2d(0, 0, new Rotation2d(0));
    }

}
