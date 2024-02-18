package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
//import org.photonvision.PhotonCamera;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Limelight;

public class IntakeSubsystem extends SubsystemBase{
    private PIDController pidController1 = new PIDController(1, 0, 0);
    private PIDController pidController2 = new PIDController(1, 0, 0);
    private Dashboard dashboard;
    //private PhotonCamera photonCamera;

    private CANSparkMax mIntakeTaker = new CANSparkMax(IntakeConstants.kIntakeTakerCanID, MotorType.kBrushless);
    private CANSparkMax mIntakeUpDown = new CANSparkMax(IntakeConstants.kIntakeUpDownCanID, MotorType.kBrushless);
    private RelativeEncoder eIntakeUpDown = mIntakeUpDown.getEncoder();
    public IntakeSubsystem(Dashboard dashboard){
        //this.photonCamera = photonCamera;
        this.dashboard = dashboard;
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
}
//roller , asa yukari + otonom