package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;



/** A command that will turn the robot to the specified angle. */
public class TurnToAngleCmd extends Command{
    private final DriveSubsystem driveSubsystem;
    private final PIDController pidController;
    private final Limelight visionSubsystem;

    public TurnToAngleCmd(DriveSubsystem driveSubsystem, Limelight visionSubsystem){
      this.driveSubsystem = driveSubsystem;
      this.visionSubsystem = visionSubsystem;
      this.pidController = new PIDController(1,0, 0);
      pidController.enableContinuousInput(-180, 180); //continous input 
      addRequirements(visionSubsystem);
      addRequirements(driveSubsystem);
    }
    @Override
    public void initialize() {
    //System.out.println("TurnToAngle Started");
      pidController.reset();
    }

    @Override
    public void execute() {
    
      double target_angle = visionSubsystem.getHorizontal();
      double current_angle = driveSubsystem.getHeading();
      double rotation_speed = pidController.calculate(current_angle, current_angle + target_angle);
      
      driveSubsystem.drive(0, 0, rotation_speed, true, true);
    }
    @Override
    public void end(boolean interrupted) {
      driveSubsystem.drive(0, 0, 0, true, true);
      //System.out.println("ended!");
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }