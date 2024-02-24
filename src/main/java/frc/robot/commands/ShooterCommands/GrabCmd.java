package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command runs the <b>roller (aka. grabber)</b>.
 * @param shooter {@link ShooterSubsystem}, which is a subsystem that includes grabber.
 */
public class GrabCmd extends Command{
    private final double kGrabSpeed = ShooterConstants.kGrabSpeed;
    private ShooterSubsystem shooter;

    public GrabCmd(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.grab(kGrabSpeed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.grab(0);
        super.end(interrupted);
    }
}
