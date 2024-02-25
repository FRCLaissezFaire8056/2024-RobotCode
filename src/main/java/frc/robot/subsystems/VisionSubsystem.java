package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private ShootCmd shootCmd = new ShootCmd(shooterSubsystem);
    private GrabCmd grabCmd = new GrabCmd(shooterSubsystem);
    private IntakeGiveCmd intakeGiveCmd = new IntakeGiveCmd(intakeSubsystem);
    private final Timer timer = new Timer(); 

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

}
