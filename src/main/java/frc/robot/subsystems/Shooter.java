package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.ShooterConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    
    private final CANSparkMax mMasterMotor = new CANSparkMax(ShooterConstants.ShooterMasterCanID, MotorType.kBrushless);
    private final CANSparkMax mFollowerMotor = new CANSparkMax(ShooterConstants.ShooterFollowerCanID, MotorType.kBrushless);
    private final CANSparkMax wristSparkMax = new CANSparkMax(ShooterConstants.WristCanID, MotorType.kBrushless);
    private final CANSparkMax mGrabber = new CANSparkMax(ShooterConstants.ShooterNeo550CanID, MotorType.kBrushless);
    private final RelativeEncoder eWrist = wristSparkMax.getEncoder();
    private Dashboard dashboard;
    public Shooter(Dashboard dashboard){
        wristSparkMax.setIdleMode(IdleMode.kBrake);
        mMasterMotor.setInverted(false);
        mFollowerMotor.setInverted(true);
        this.dashboard = dashboard;

    }
    
    public void wristMove(double speed){
        wristSparkMax.set(speed);
    }

    public RelativeEncoder giveWristEncoder(){
        return eWrist;
    }

    public void shoot(){
        mMasterMotor.set(ShooterConstants.defaultSpeed);
        mFollowerMotor.set(ShooterConstants.defaultSpeed);
        mGrabber.set(ShooterConstants.defaultSpeed);
    }
    public void stop(){
        mMasterMotor.set(0);
        mFollowerMotor.set(0);
        mGrabber.set(0);
    }
    public void grab(double speed){
        mGrabber.set(speed);
    }
}
