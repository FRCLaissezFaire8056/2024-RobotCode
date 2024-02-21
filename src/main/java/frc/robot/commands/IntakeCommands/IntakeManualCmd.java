package frc.robot.commands.IntakeCommands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class IntakeManualCmd extends Command{
    private IntakeSubsystem intakeSubsystem;
    private RelativeEncoder eIntake;

    private final double dummyIntakeEncoderValueDown = 60.5;
    private final double IntakeSpeedUp = 0.5;
    private final double IntakeSpeedDown = -0.5;

    public IntakeManualCmd(Shooter shooter, IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.eIntake = intakeSubsystem.giveUpDownEncoder();
    }
    
    @Override
    public void execute() {
        
        this.intakeSubsystem.IntakeTake(0.5);
        while(eIntake.getPosition() < dummyIntakeEncoderValueDown){this.intakeSubsystem.IntakeMove(IntakeSpeedDown);}
        this.intakeSubsystem.IntakeMove(0);
        super.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.IntakeTake(0);
        while(eIntake.getPosition() > 0){this.intakeSubsystem.IntakeMove(IntakeSpeedUp);}
        this.intakeSubsystem.IntakeMove(0);
        super.end(interrupted);
    }

}
