package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.ShooterConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{
    
    private final CANSparkMax mMasterMotor = new CANSparkMax(ShooterConstants.ShooterMasterCanID, MotorType.kBrushless);
    private final CANSparkMax mFollowerMotor = new CANSparkMax(ShooterConstants.ShooterFollowerCanID, MotorType.kBrushless);
    private final CANSparkMax mGrabber = new CANSparkMax(ShooterConstants.ShooterNeo550CanID, MotorType.kBrushless);
    private Dashboard dashboard;
    public Shooter(Dashboard dashboard){
        mMasterMotor.setInverted(false);
        mFollowerMotor.setInverted(true);
        this.dashboard = dashboard;

    }
    
    public void shoot(){
        mMasterMotor.set(ShooterConstants.defaultSpeed);
        mFollowerMotor.set(ShooterConstants.defaultSpeed);
    }
    public void stop(){
        mMasterMotor.set(0);
        mFollowerMotor.set(0);
    }
    public void grab(double speed){
        mGrabber.set(speed);
    }
}
