package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeCmd extends Command{
    private Shooter shooter;
    private IntakeSubsystem intakeSubsystem;
    private RelativeEncoder eWrist;
    private RelativeEncoder eIntake;

    private final double dummyIntakeEncoderValueDown = 60.5;
    private final double dummyWristEncoderValueDown = 60.5;

    private final double dummyIntakeEncoderValueUp = 60.5;
    private final double dummyWristEncoderValueUp = 60.5;

    private final double wristSpeedUp = 0.5;
    private final double wristSpeedDown = -0.5;

    private final double IntakeSpeedUp = 0.5;
    private final double IntakeSpeedDown = -0.5;

    public IntakeCmd(Shooter shooter, IntakeSubsystem intakeSubsystem){
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.eWrist = shooter.giveWristEncoder();
        this.eIntake = intakeSubsystem.giveUpDownEncoder();
    }
    
    /*
     *  intake taker +yon don
     *  intake asagi
     *  nesneye ilerle -- atla
     *  wrist asagi
     *  intake yukari tukur
     *  grab
     *  shooterin her seyi donuyor
     *  bitti
     */
    @Override
    public void execute(){
        super.execute();
        //intake asagi indi calisiyor
        intakeSubsystem.IntakeTake(0.5);
        while(eIntake.getPosition() < dummyIntakeEncoderValueDown){intakeSubsystem.IntakeMove(IntakeSpeedDown);}
        intakeSubsystem.IntakeMove(0);

        //photonvision kismi
        /*
         * 
         * yazilacak
         * 
         */
        //wrist geriye yanasti, intake yukari ve grab
        intakeSubsystem.IntakeMove(0);


        while(eWrist.getPosition() < dummyWristEncoderValueDown){shooter.wristMove(wristSpeedDown);}
        shooter.wristMove(0);
        intakeSubsystem.IntakeMove(IntakeSpeedUp);
        

        //grab ve shooter hedeflendi
        shooter.grab(1.0);
        while(eWrist.getPosition() > dummyIntakeEncoderValueUp){shooter.wristMove(wristSpeedUp);}
        
        //shooter don
        shooter.shoot();


    }
}
