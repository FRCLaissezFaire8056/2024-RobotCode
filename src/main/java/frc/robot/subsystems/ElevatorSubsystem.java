package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax mElevatorMaster = new CANSparkMax(ElevatorConstants.kMasterCanID, MotorType.kBrushless);
    private final CANSparkMax mElevatorFollower = new CANSparkMax(ElevatorConstants.kFollowerCanID, MotorType.kBrushless);
    private final PIDController pidController = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);
    private final SlewRateLimiter filter = new SlewRateLimiter(ElevatorConstants.kElevatorSlewRate);
    private final RelativeEncoder masterEncoder;

    public ElevatorSubsystem(){
        mElevatorMaster.follow(mElevatorFollower);
        //pidController.enableContinuousInput(-180, 180);
        masterEncoder = mElevatorFollower.getEncoder();
    }


    public double elevatorState(){
        double encoder_reading = mElevatorMaster.getEncoder().getPosition();
        double state = encoder_reading * 1;
        return state;
    }

    public void ElevatorPID(double goal){
        double speed = pidController.calculate(elevatorState(), goal);
        mElevatorMaster.set(speed);
        mElevatorFollower.set(-speed);
    }
    public void move(double speed){
        speed = filter.calculate(speed);
        mElevatorMaster.set(speed);
        mElevatorFollower.set(-speed);
    }



    public void driveWithJoystick(double joystick){
        move(joystick);
    }

    public double returnValueForEntry(){
        return masterEncoder.getPosition();
    }

    @Override
    public void periodic() {
        //dashboard.putOnDashboard("emtemt", mElevatorMaster.getEncoder().getPosition(), 2);
    }
}
