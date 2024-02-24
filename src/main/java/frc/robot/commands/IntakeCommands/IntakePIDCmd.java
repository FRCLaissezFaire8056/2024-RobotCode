package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.custom.CustomEncoder;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This command has 2 parameters:
 * @param shooter {@link IntakeSubsystem} class, which is <b>subsystem</b> for intake mechanism.
 * @param goal Desired position of <b>intake</b>
 * 
 * @apiNote This command contains {@link PIDController#reset()} from {@link CustomEncoder} class
 */
public class IntakePIDCmd extends Command {
    private IntakeSubsystem intakeSubsystem;
    
    private final double kP = IntakeConstants.kP;
    private final double kI = IntakeConstants.kI;
    private final double kD = IntakeConstants.kD;

    private final double kRad = IntakeConstants.kRad;

    private double goal;

    private CustomEncoder customEncoder;

    public IntakePIDCmd(IntakeSubsystem intakeSubsystem, double goal){
        this.intakeSubsystem = intakeSubsystem;
        this.goal = goal;
        customEncoder = new CustomEncoder(intakeSubsystem.giveMoverEncoder(), kRad);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double speed = customEncoder.calculatePID(kP, kI, kD, goal);
        intakeSubsystem.limitlessMove(speed);
        super.execute();
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.limitlessMove(0);
        super.end(interrupted);
    }
}
