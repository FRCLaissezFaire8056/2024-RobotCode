package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeCmd extends Command{
    private Shooter shooter;
    private IntakeSubsystem intakeSubsystem;
    public IntakeCmd(Shooter shooter, IntakeSubsystem intakeSubsystem){
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
    }
    
    public void IntakeTake(){
        
    }
}
