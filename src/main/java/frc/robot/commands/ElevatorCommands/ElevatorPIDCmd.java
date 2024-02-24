package frc.robot.commands.ElevatorCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.custom.CustomEncoder;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCmd extends Command {

    private final double kP = ElevatorConstants.kP;
    private final double kI = ElevatorConstants.kI;
    private final double kD = ElevatorConstants.kD;

    private final double kRad = ElevatorConstants.kRad;

    private double goal;

    private ElevatorSubsystem elevatorSubsystem;
    private CustomEncoder customEncoder;

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double goal){
        this.elevatorSubsystem = elevatorSubsystem;
        this.goal = goal;
        customEncoder = new CustomEncoder(elevatorSubsystem.giveEncoder(), kRad);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double speed = customEncoder.calculatePID(kP, kI, kD, goal);
        elevatorSubsystem.limitlessMove(speed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.limitlessMove(0);
        super.end(interrupted);
    }
}
