// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
//Commands
import frc.robot.commands.ShooterCommands.ShootCmd;
import frc.robot.commands.ShooterCommands.GrabCmd;
import frc.robot.commands.IntakeCommands.IntakeGiveCmd;
import frc.robot.commands.IntakeCommands.IntakeTakeCmd;
import frc.robot.commands.ShooterCommands.WristPIDCmd;
import frc.robot.commands.DriveCommands.SetXCmd;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handle    d in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  //private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kCTRE);


  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final Dashboard dashboard = new Dashboard(pdp, m_robotDrive.m_gyro);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  //private final NewVisionSubsystem visionSubsystem = new NewVisionSubsystem();

  // The driver's controller

  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick js =  new Joystick(0);
  Joystick js2 = new Joystick(1);
  Robot robot = new Robot();
  /**  
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    intakeSubsystem.setDefaultCommand(
        new RunCommand(
                () -> intakeSubsystem.driveWithJoystick(
                        js.getRawAxis(1)), 
                intakeSubsystem)
            ); 


    elevatorSubsystem.setDefaultCommand(
        new RunCommand(
            () -> elevatorSubsystem.driveWithJoystick(
                        (js.getRawAxis(3) - js.getRawAxis(2))),
                elevatorSubsystem)
            );


    shooter.setDefaultCommand(
        new RunCommand(
            () -> shooter.driveWithJoystick(
                    js.getRawAxis(4)),
                shooter)
            );
    
    

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(                                             //!!! PORT 1 !!!
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(js2.getRawAxis(1)/2, OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(js2.getRawAxis(0)/2, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((js2.getRawAxis(3) - js2.getRawAxis(2))/2, OIConstants.kDriveDeadband),
                

                true, true),
            m_robotDrive));
    
    
}


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(js, 1)
        .whileTrue(new IntakeTakeCmd(intakeSubsystem));
    
    new JoystickButton(js, 2)
        .whileTrue(new IntakeGiveCmd(intakeSubsystem));
    
    new JoystickButton(js, 3)
        .whileTrue(new SetXCmd(m_robotDrive)); 

    new JoystickButton(js, 4)
        .whileTrue(new ShootCmd(shooter));
    
    new JoystickButton(js, 5)
        .whileTrue(new GrabCmd(shooter));
    


        
    
    new JoystickButton(js2, 1)
        .whileTrue(new WristPIDCmd(shooter, 2000));
    
    //new JoystickButton(js2, 2)
        //.whileTrue(m_robotDrive.goToPose2d(visionSubsystem.getPose2d()));

    new JoystickButton(js2, 5)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.resetRelative(), m_robotDrive));
    
    SmartDashboard.putData("on-the-fly", Commands.runOnce(() -> {
        Pose2d currPose2d = m_robotDrive.getPose();

        Pose2d startPose2d = new Pose2d(currPose2d.getTranslation(), new Rotation2d());
        //Pose2d endPose2d = new Pose2d(currPose2d.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
        Pose2d visionPose = visionSubsystem.getPose2d();
        Pose2d endPose2d = new Pose2d(currPose2d.getTranslation().plus(visionPose.getTranslation()), new Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPose2d, endPose2d);
        SmartDashboard.putString("bezier", bezierPoints.toString());
        SmartDashboard.putString("endpose", endPose2d.toString());

        PathPlannerPath path = new PathPlannerPath(
            bezierPoints, 
            new PathConstraints(1.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, currPose2d.getRotation())
        );
        
        path.preventFlipping = true;
        SmartDashboard.putString("path", path.toString());
        AutoBuilder.followPath(path).schedule();
    }));
  }
  /**
   * Just for disappearing the error: <b>m_robotContainer not used</b>
   */
  public void errorSolver(){}


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
