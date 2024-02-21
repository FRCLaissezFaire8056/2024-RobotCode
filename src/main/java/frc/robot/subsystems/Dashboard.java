package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import frc.robot.Constants.ShuffleBoardConstants;

public class Dashboard extends SubsystemBase{
    
    private final PowerDistribution pdp;
    private final AHRS m_gyro;
    public Dashboard(PowerDistribution pdp, AHRS m_gyro){//, AHRS m_gyro){
        this.pdp = pdp;
        this.m_gyro = m_gyro;
    }
    //private PowerDistribution pdp = new PowerDistribution();
    private ShuffleboardTab driverTab = Shuffleboard.getTab(ShuffleBoardConstants.kTabName);

    
    private ComplexWidget pdpWidget;
    private ComplexWidget gyroWidget;

    protected void putOnDashboard(String key, double number, int type){
        switch(type){
            case 1://text view
                Shuffleboard.getTab(ShuffleBoardConstants.kTabName)
                            .add(key, number)
                            .withWidget(BuiltInWidgets.kField)
                            .getEntry();
            break;

            case 2://simple dial
                Shuffleboard.getTab(ShuffleBoardConstants.kTabName)
                            .add(key, number)
                            .withWidget(BuiltInWidgets.kDial)
                            .getEntry();

            case 3://number bar
                Shuffleboard.getTab(ShuffleBoardConstants.kTabName)
                            .add(key, number)
                            .withWidget(BuiltInWidgets.kNumberBar)
                            .getEntry();
            break;

            default:
                SmartDashboard.putNumber(key, number);
            break;
                        
        }
    }

    protected void putPDPWidget(){
        this.pdpWidget = driverTab
            .add("PDP", pdp)
            .withWidget(BuiltInWidgets.kPowerDistribution)
            .withPosition(0, 7)
            .withSize(3, 3);
    }

    protected void putGyroAngle(){
        this.gyroWidget = driverTab
            .add("Gyro", m_gyro)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(12, 12)
            .withSize(2, 2);
    }
}
