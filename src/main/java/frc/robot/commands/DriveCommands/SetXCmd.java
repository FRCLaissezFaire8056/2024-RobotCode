package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Sets wheels to X position.
 * @param DriveSubsystem {@link DriveSubsystem} for driving.
 */
public class SetXCmd extends Command{
    private DriveSubsystem driveSubsystem;
    public SetXCmd(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
    }
    
    @Override
    public void execute() {
        driveSubsystem.setX();
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
        super.end(interrupted);
    }
}
