package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;


public class AprilTagAimer extends Command{
    private final DriveSubsystem driveSubsystem;
    private final PIDController pidController;
    private final Limelight visionSubsystem;

    public AprilTagAimer(DriveSubsystem driveSubsystem, Limelight visionSubsystem){
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pidController = new PIDController(1, 0, 0);
        pidController.enableContinuousInput(-360, 360);
        addRequirements(driveSubsystem);
    }

    private void TurnToTarget(){
        double angle = pidController.calculate(visionSubsystem.getHorizontal(), 0);
        driveSubsystem.drive(0, 0, angle, false, false);
    }
    private void GoToTarget(){
        driveSubsystem.drive(pidController.calculate(visionSubsystem.getHorizontal()), 0, 0, false, false);
    }
    public void AimToTarget(){
        TurnToTarget();
        GoToTarget();
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public void execute() {
        AimToTarget();
        super.execute();
    }

}
