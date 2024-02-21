package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

public class ShooterManualCmd extends Command{
    
    private Shooter shooter;
    private IntakeSubsystem intakeSubsystem;

    private final double outTakeSpeed = -.5;

    public ShooterManualCmd(Shooter shooter, IntakeSubsystem intakeSubsystem){
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        this.shooter.shoot();
        this.intakeSubsystem.IntakeTake(outTakeSpeed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.IntakeTake(0);
        this.shooter.stop();
        super.end(interrupted);
    }
}
