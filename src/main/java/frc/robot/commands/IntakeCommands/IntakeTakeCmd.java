package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTakeCmd extends Command{
    private final  double kTake = IntakeConstants.kTake;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeTakeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        intakeSubsystem.setRoller(kTake);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRoller(0);
        super.end(interrupted);
    }
}
