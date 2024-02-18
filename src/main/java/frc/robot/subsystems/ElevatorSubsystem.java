package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private CANSparkMax mElevatorMaster = new CANSparkMax(ElevatorConstants.kMasterCanID, MotorType.kBrushless);
    private CANSparkMax mElevatorFollower = new CANSparkMax(ElevatorConstants.kFollowerCanID, MotorType.kBrushless);
    private PIDController pidController = new PIDController(1, 0, 0);
    private RelativeEncoder masterEncoder;
    private Dashboard dashboard;
    public ElevatorSubsystem(Dashboard dashboard){
       
        masterEncoder = mElevatorFollower.getEncoder();
        this.dashboard = dashboard;
    }

    public void MoveUp(double speed){
        mElevatorMaster.set(speed);
        mElevatorFollower.set(-speed);
    }
    public void MoveDown(double speed){
        mElevatorMaster.set(speed);
    }
    public void testStart(double speed){
        mElevatorMaster.set(speed);
        mElevatorFollower.set(-speed);
    }
    public void testStop(){
        mElevatorMaster.set(0);
    }
    @Override
    public void periodic() {
        //dashboard.putOnDashboard("emtemt", mElevatorMaster.getEncoder().getPosition(), 2);
    }
}
