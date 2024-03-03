package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.IntakeCommands.IntakeGiveCmd;
import frc.robot.commands.ShooterCommands.GrabCmd;
import frc.robot.commands.ShooterCommands.ShootCmd;

import org.photonvision.PhotonCamera;
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

    private final PhotonCamera camera = new PhotonCamera("targetcam");
    private final PIDController turningController = new PIDController(kTurningP, kTurningI, kTurningD);
    private final PIDController drivingController = new PIDController(kDrivingP, kDrivingI, kDrivingD);

    private DriveSubsystem driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        camera.setPipelineIndex(0);
    }

    public void getCloserToTag(double distance, int Id){
        var result = camera.getLatestResult();
        if(result.hasTargets() && result.getBestTarget().getFiducialId() == Id){
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                kCamHeight, 
                kAmpHeight, 
                kCamAngle, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            drivingSpeed = drivingController.calculate(range, distance);
            turningSpeed = turningController.calculate(result.getBestTarget().getYaw(), 0.2);
        }
        else{
            turningSpeed = 0;
            drivingSpeed = 0;
        }

        //System.out.println("turning " + turningSpeed + "\n" + "driving " + drivingSpeed);
        SmartDashboard.putNumber("drivingSpeed", drivingSpeed);
        SmartDashboard.putNumber("turning", turningSpeed);

        driveSubsystem.drive(drivingSpeed, 0, turningSpeed, false, false);
        //driveSubsystem.drive(0, 0, turningSpeed, false, false);
    }

    public void selectId(int Id){
        switch (Id) {
            case 1:
                getCloserToTag(1, 1);
                break;
            
            case 8:
                getCloserToTag(1.4, 8);
            default:
                break;
        }
    }

}