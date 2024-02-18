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

    private CANSparkMax m1 = new CANSparkMax(IntakeConstants.kIntakeMotor1CanID, MotorType.kBrushless);

    private CANSparkMax m2 = new CANSparkMax(IntakeConstants.kIntakeMotor2CanID, MotorType.kBrushless);

    public IntakeSubsystem(Dashboard dashboard){
        //this.photonCamera = photonCamera;
        this.dashboard = dashboard;
    }

    public void IntakeMove(double speed){
        m1.set(speed);
    }
      public void IntakeTake(double speed){
        m2.set(speed);
    }
}
