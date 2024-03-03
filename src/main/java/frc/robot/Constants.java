// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.4;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 53;
    public static final int kRearLeftDrivingCanId = 54;
    public static final int kFrontRightDrivingCanId = 52;
    public static final int kRearRightDrivingCanId = 51;

    public static final int kFrontLeftTurningCanId = 43;
    public static final int kRearLeftTurningCanId = 44;//ssss
    public static final int kFrontRightTurningCanId = 42;
    public static final int kRearRightTurningCanId = 41;

    public static final boolean kGyroReversed = true;
  }

  public static final class AutoBuilderConstants {
    public static final HolonomicPathFollowerConfig hPathConf =
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kTurnToleranceDeg = 2;
    public static final double kTurnRateToleranceDegPerS = 0;

  }

  public static final class LimelightConstants {
    public static final double limelightMountAngleDegrees = 15.0;
    public static final double limelightLensHeightInches = 45.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Auto Trajectory 
    public static final String trajectoryJSON = "";

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShuffleBoardConstants {
    public static final int kPDPId = 0;
    public static final String kTabName = "Control";

  }
  public static final class ShooterConstants {
    public static final int kMasterCanId = 36;
    public static final int kFollowerCanId = 26;
    public static final int kGrabberCanId = 48;
    public static final int kWristCanId = 16;

    public static final double kDefaultSpeed = 1.0;
    public static final double kGrabSpeed = 1.0;

    public static final double kSlewLimit = 1.6;
    public static final double kLimitUp = 8000;//MARK
    public static final double kLimitDown = -60;//MARK

    public static final double kGoalUp = 4430;
    public static final double kGoalDown = 0;


    public static final double kP = 0.0008;
    public static final double kI = 0.000001;
    public static final double kD = 0.000018;

  

    public static final double kRad = 21.5;

  }
  
  public static final class ElevatorConstants {
    public static final int kMasterCanID = 31;
    public static final int kFollowerCanID = 32;
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0.000018;
    public static final double kSlewLimit = 1;

    public static final double kLimitUp = 500;//MARK
    public static final double kLimitDown = 5;//MARK

    public static final double kGoalUp = 500;
    public static final double kGoalDown = 0;

    public static final double kRad = 31.75;
  }

  public static final class IntakeConstants {
    public static final int kRollerCanId = 18;
    public static final int kMoverCanId = 17;
    public static final double kSlewLimit = 0.8; 

    public static final double kLimitUp = 340.0;//MARK
    //public static final double kLimitDown = 0;//MARK
    public static final double kLimitDown = -1000000;

    public static final double kGoalUp = 340.0;
    public static final double kGoalDown = 0;


    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0.0015;

    public static final double kRad = 18.5;

    public static final double kTake = .8;
    public static final double kGive = -1.0;

    public static final double kHumanPlayerPos = 100; //POSE
    public static final double kFloorPos = 0;         //POSE
    public static final double kShootPos = 200;       //POSE
  }



  public static final class VisionConstants {
    public static final double kAmpHeight = 1.3;//Units.inchesToMeters(50.13);//50.13
    public static final double kCamHeight = 0.675;//68.5
    public static final double kCamAngle = Units.degreesToRadians(25);

    public static final double kCamZToCenter = 0.675;
    public static final double kCamXToCenter = 0.280;
    public static final boolean kCamForvard = false;

    public static final double kTurningP = 0.01;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.001;

    public static final double kDrivingP = 0.12;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0.01;

    public static final Transform3d robotToCam = new Transform3d(new Translation3d(kCamXToCenter, 0, kCamZToCenter), new Rotation3d(0, 180, 15));
  }
}
