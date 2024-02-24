package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.custom.CustomEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase{

    private final int kMasterCanId = ElevatorConstants.kMasterCanID;
    private final int kFollowerCanId = ElevatorConstants.kFollowerCanID;

    private final double kSlewLimit = ElevatorConstants.kSlewLimit;
    private final double kRad = ElevatorConstants.kRad;

    private final double kLimitUp = ElevatorConstants.kLimitUp;
    private final double kLimitDown = ElevatorConstants.kLimitDown;

    private final CANSparkMax mElevatorMaster = new CANSparkMax(kMasterCanId, MotorType.kBrushless);
    private final CANSparkMax mElevatorFollower = new CANSparkMax(kFollowerCanId, MotorType.kBrushless);
    
    private final SlewRateLimiter filter = new SlewRateLimiter(kSlewLimit);
    private final CustomEncoder customEncoder = new CustomEncoder(mElevatorMaster.getEncoder(), kRad);
 
    public ElevatorSubsystem(){}
    /**
     * Starts the movement of the elevator at given speed. 
     * <b>Don't use if you know what are you doing.</b>
     * @param speed
     *      (+)  -> Up
     *      (-)  -> Down
     */ 
    public void limitlessMove(double speed){
        mElevatorMaster.set(speed);
        mElevatorFollower.set(-speed);
    }

    /**
     * Starts the movement of the elevator at given speed
     * @param speed
     *      (+)  -> Up
     *      (-)  -> Down
     */ 
    public void move(double speed){
        speed = filter.calculate(speed);
        double dist = customEncoder.getDistance();
        if((speed > 0 && dist <= kLimitUp) || (speed < 0 && dist >= kLimitDown))
        {
            mElevatorMaster.set(speed);
            mElevatorFollower.set(-speed);
        }
        else{
            mElevatorMaster.set(0);
            mElevatorFollower.set(0);
        }

    }

    public RelativeEncoder giveEncoder(){
        return mElevatorMaster.getEncoder();
    }

    public void driveWithJoystick(double joystick){
        move(joystick);
    }
    @Override
    public void periodic() {
         SmartDashboard.putNumber("encoder-elevator", customEncoder.getDistance());
    }
}
