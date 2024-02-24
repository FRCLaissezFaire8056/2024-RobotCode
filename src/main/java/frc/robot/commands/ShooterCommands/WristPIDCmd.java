package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.custom.CustomEncoder;

/**
 * This command has 2 parameters:
 * @param shooter {@link ShooterSubsystem} class, which is <b>subsystem</b> for shooter mechanism.
 * @param goal Desired position of <b>wrist</b>
 * 
 * @apiNote This command contains {@link PIDController#reset()}
 */
public class WristPIDCmd extends Command{
    private ShooterSubsystem shooter;

    private final double kP = ShooterConstants.kP;
    private final double kI = ShooterConstants.kI;
    private final double kD = ShooterConstants.kD;
    
    private final double kRad = ShooterConstants.kRad;

    private double goal;

    private CustomEncoder customEncoder;
    

    public WristPIDCmd(ShooterSubsystem shooter, double goal){
        this.shooter = shooter;
        this.goal = goal;
        customEncoder = new CustomEncoder(shooter.giveWristEncoder(), kRad);
        
    }      
    
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double speed = customEncoder.calculatePID(kP, kI, kD, goal);
        shooter.wristSet(speed);
        super.execute();
    }
    @Override
    public void end(boolean interrupted) {
        shooter.wristSet(0);
        super.end(interrupted);
    }
}
