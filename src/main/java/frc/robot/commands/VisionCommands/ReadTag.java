package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ReadTag extends Command{
    private VisionSubsystem visionSubsystem;
    private DriveSubsystem driveSubsystem;
    
    public ReadTag(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem){
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        var results = visionSubsystem.camera.getLatestResult();
        Pose2d robotPose2d;
        //System.out.print(results.getBestTarget());
        if(results.hasTargets()){
            var updt = visionSubsystem.photonPoseEstimator.update();
            //System.out.println(updt);
            //System.out.println("-----------------------------------");
            Pose3d currPose3d = updt.get().estimatedPose;
            //System.out.println(currPose3d);
            //System.out.println("-----------------------------------");
            robotPose2d = currPose3d.toPose2d();
            System.out.println(robotPose2d);
            System.out.println("-----------------------------------");
            
        }
        else robotPose2d = driveSubsystem.getPose();
        
        driveSubsystem.followTrajectory(robotPose2d, 1.0);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("bitii");
        super.end(interrupted);
    }
}
