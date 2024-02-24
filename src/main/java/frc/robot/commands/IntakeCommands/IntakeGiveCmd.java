package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeGiveCmd extends Command{
    private final double kGive = IntakeConstants.kGive; 
    private final IntakeSubsystem intakeSubsystem;

    public IntakeGiveCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        intakeSubsystem.setRoller(kGive);
        super.execute();
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRoller(0);
        super.end(interrupted);
    }
}
