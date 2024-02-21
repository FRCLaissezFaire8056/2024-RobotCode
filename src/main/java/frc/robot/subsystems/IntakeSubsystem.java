package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

//import org.photonvision.PhotonCamera;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final PIDController pidController1 = new PIDController(1, 0, 0);
    private final SlewRateLimiter filter = new SlewRateLimiter(IntakeConstants.kIntakeSlewRate);
    //private PhotonCamera photonCamera;

    private CANSparkMax mIntakeTaker = new CANSparkMax(IntakeConstants.kIntakeTakerCanID, MotorType.kBrushless);
    private CANSparkMax mIntakeUpDown = new CANSparkMax(IntakeConstants.kIntakeUpDownCanID, MotorType.kBrushless);
    private RelativeEncoder eIntakeUpDown = mIntakeUpDown.getEncoder();
    public IntakeSubsystem(){
        //this.photonCamera = photonCamera;
        mIntakeUpDown.setIdleMode(IdleMode.kBrake);
    }
    /*
     * (-) yon yukari
     * (+) yon asagi      
     */
    public void IntakeMove(double speed){
        mIntakeTaker.set(speed);
    }
    /*
     * (+) yon = in 
     * (-) yon = out
     */ 
    public void IntakeTake(double speed){
        mIntakeUpDown.set(speed);
    }

    public RelativeEncoder giveUpDownEncoder(){
        return eIntakeUpDown;
    }

    public double returnValueForEntry(){
        return eIntakeUpDown.getPosition();
    }

    public void driveWithJoystick(double joystick){
        IntakeMove(joystick);
    }
}
//roller , asa yukari + otonom