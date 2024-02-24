package frc.deprecated;
import java.lang.Math;

public class EncoderUtils {
    public static double toDistance(double rotations, double radius, double gearRatio){
        return Math.PI * 2 * radius * rotations;
    }
}
