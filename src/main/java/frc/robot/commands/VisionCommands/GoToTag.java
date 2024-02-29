package frc.robot.commands.VisionCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GoToTag extends InstantCommand{

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;

    public GoToTag(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d startPose = new Pose2d(
            currentPose.getTranslation(), new Rotation2d());
        Pose2d visionPose = visionSubsystem.getPose2d();
        Pose2d endPose = new Pose2d(
            currentPose.getTranslation().plus(new Translation2d(-(visionPose.getX()-2.0), visionPose.getY())),
            new Rotation2d()
        );

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose, endPose);

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(
                4.0, 
                4.0, 
                Units.degreesToRadians(360), 
                Units.degreesToRadians(540)),
            new GoalEndState(0.0, currentPose.getRotation()));

        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();
        super.execute();
    }
}
