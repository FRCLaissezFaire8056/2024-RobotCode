package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ElbowConstants;

public class ElbowSubsystem extends SubsystemBase{
    private final CANSparkMax elbowSparkMax = new CANSparkMax(ElbowConstants.kElbowCanID, MotorType.kBrushless);
    private Dashboard dashboard;
    public ElbowSubsystem(Dashboard dashboard){
        this.dashboard = dashboard;
    }

    public void move(double speed){
        elbowSparkMax.set(speed);
    }
}
