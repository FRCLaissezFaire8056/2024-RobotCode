package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command runs the <b>shooter wheels</b>.
 * @param shooter {@link ShooterSubsystem}, which is a subsystem that includes grabber.
 */
public class ShootCmd extends Command{
    private final double kDefaultSpeed = ShooterConstants.kDefaultSpeed;

    private ShooterSubsystem shooter;

    public ShootCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.shoot(kDefaultSpeed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        super.end(interrupted);
    }
}
