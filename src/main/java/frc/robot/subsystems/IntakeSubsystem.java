package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.custom.CustomEncoder;

public class IntakeSubsystem extends SubsystemBase{
    private final int kRollerCanId = IntakeConstants.kRollerCanId;
    private final int kMoverCanId = IntakeConstants.kMoverCanId;

    private final double kRad = IntakeConstants.kRad;

    private final double kSlewLimit = IntakeConstants.kSlewLimit;

    private final double kLimitUp = IntakeConstants.kLimitUp;
    private final double kLimitDown = IntakeConstants.kLimitDown;

    private final SlewRateLimiter filter = new SlewRateLimiter(kSlewLimit);

    private CANSparkMax mRoller = new CANSparkMax(kRollerCanId, MotorType.kBrushless);
    private CANSparkMax mMover = new CANSparkMax(kMoverCanId, MotorType.kBrushless);

    private CustomEncoder customEncoder;

    public IntakeSubsystem(){
        customEncoder = new CustomEncoder(mMover.getEncoder(), kRad);
    }

    /**
     * Starts the movement of the intake at given speed
     * @param speed
     */ 
    public void limitlessMove(double speed){
        mMover.set(speed);
    }

    /**
     * Starts the movement of the roller at given speed
     * @param speed
     *      *(+)  -> In
     *      *(-)  -> Out
     */ 
    public void setRoller(double speed){
        mRoller.set(speed);
    }

    /**
     * Moves the Intake at up and down direction.
     * @param speed
     *      *(-)  -> Up
     *      *(+)  -> Down      
     */
    public void move(double speed){
        speed = filter.calculate(speed);
        double distance = customEncoder.getDistance();
        if((speed > 0 && distance <= kLimitUp) || (speed < 0 && distance >= kLimitDown))//MARK
            limitlessMove(speed);
        else
            limitlessMove(0);
    }

    public RelativeEncoder giveMoverEncoder(){
        return mMover.getEncoder();
    }

    public void driveWithJoystick(double joystick){
        move(joystick);
    }

      @Override
    public void periodic() {
         SmartDashboard.putNumber("encoder-intake", customEncoder.getDistance());
    }
}
