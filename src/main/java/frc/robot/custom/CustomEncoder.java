package frc.robot.custom;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

/**
 * <b>Customized</b> version of {@link RelativeEncoder}.
 * This Class contains extra psedo-reset system and tools for conversion.
 */
public class CustomEncoder{

    private RelativeEncoder relativeEncoder;
    private final PIDController pidController = new PIDController(0, 0, 0);
    private double kRad;
    
    private double offset;
    private double currentRot;

    /**
     * 
     * @param relativeEncoder {@link RelativeEncoder} of desired motor
     * @param kRad Radius of Gear(Wheel)
     */
    public CustomEncoder(RelativeEncoder relativeEncoder, double kRad){
        this.relativeEncoder = relativeEncoder;
        this.kRad = kRad;
        
        pidController.reset();
        offset = relativeEncoder.getPosition();
    }

    //Static Methods
    /**
     * It converts rotations to (mili)metric value
     * @param rotations  {@link RelativeEncoder#getPosition()}
     * @param radius     Radius of Gear(Wheel)
     * @return Gear perimeter divided gear ratio
     */
    public static double toDistance(double rotations, double radius){
        return (Math.PI * 2 * radius * rotations);
    }

    //Private Methods
    /**
     * It calculates the position by subtracting real current positon by offset.
     * @return Same as {@value #modifiedRot} current rotation - offset rotation.
     */
    private double calcPosition(){
        currentRot = relativeEncoder.getPosition();
        return currentRot - offset;
    }

    //Public Methods
    /**
     * Sets offset to current encoder value gathered from {@link RelativeEncoder#getPosition()}.
     * @return Old Offset Value
     */
    public double reset(){
        double temp = offset;
        offset = relativeEncoder.getPosition();
        return temp;
    }

    /**
     * Calculates the position and returns it.
     */
    public double getPosition(){
        return calcPosition();
    }


    /**
     * It gives converted rotation value.
     * @return Position in (mili)meter(s).
     */
    public double getDistance(){
        return toDistance(getPosition(), kRad);
    }

    /**
     * This function gives <b>calculated PID</b> values
     * which is suitable for {@link CANSparkMax#set()} function's <b>speed</b> parameter.
     * 
     * @param kP <b>kP</b> value for {@link PIDController}
     * @param kI <b>kI</b> value for {@link PIDController}
     * @param kD <b>kD</b> value for {@link PIDController}
     * @param kGoal setpoint value for {@link PIDController}
     * @return {@link PIDController#calculate} with {@value #getDistance()}, {@value kGoal}
     */
    public double calculatePID(double kP, double kI, double kD, double kGoal){
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);

        return pidController.calculate(getDistance() ,kGoal);
    }

}
