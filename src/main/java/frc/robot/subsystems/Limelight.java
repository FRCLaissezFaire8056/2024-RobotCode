package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;


public class Limelight extends SubsystemBase {
  private Dashboard dashboard;
  public Limelight(Dashboard dashboard){
    this.dashboard = dashboard;
  }
  
  //Limelight NetworkTables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  double tid[] = table.getEntry("tid").getDoubleArray(new double[6]);

  public double x;
  public double y;
  public double state; 
  public double area;
  public double distance;

  //whether the limelight has any valid targets(0 or 1)
  public double getState(){
    return state;
  }
  //get x angle 
  public double getHorizontal(){
    return x;
  }
  //get y angle 
  public double getVertical(){
    return y;
  }
  //get y angle 
  public double getArea(){
    return y;
  }
  public double[] getID(){
    return tid;
  }
  //get distance 
  public double getDistance(double verticalAngle, double mountAngle, double limelightHeight, double goalHeight){

    double angleToGoalDegrees = verticalAngle + mountAngle; 
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distance = (goalHeight - limelightHeight) / Math.tan(angleToGoalRadians);
    return distance;
  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    state = tv.getDouble(0.0);
    area = ta.getDouble(0.0);
    distance = getDistance(y, LimelightConstants.limelightMountAngleDegrees, LimelightConstants.limelightLensHeightInches, 0);
  }
}

