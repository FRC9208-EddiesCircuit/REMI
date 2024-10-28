// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotMechSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
*/
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final RobotMechSubsystem robotMechSubsystem = new RobotMechSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick TwistJoystick = new Joystick(0);
  private final Joystick DriveJoystick = new Joystick(2);
  public final static XboxController RobotMechJoystick = new XboxController(OIConstants.kRobotMechControllerPort);

  private final SendableChooser<Command> m_chooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


  //Register Named Commands
  NamedCommands.registerCommand("AutoIntakeCmd", new AutoIntakeCmd(robotMechSubsystem));//.withTimeout(5));
  NamedCommands.registerCommand("AutoShooterCmd", new AutoShooterCmd(robotMechSubsystem).withTimeout(5));
  NamedCommands.registerCommand("AutoShooterCmd2", new AutoShooterCmd2(robotMechSubsystem).withTimeout(5));
  NamedCommands.registerCommand("SetZeroHeadingCmd", new SetZeroHeadingCmd(swerveSubsystem));


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -DriveJoystick.getRawAxis(1),//OIConstants.kDriverYAxis
      () -> -DriveJoystick.getRawAxis(0),//OIConstants.kDriverXAxis
      () -> -TwistJoystick.getRawAxis(2),//OIConstants.kDriverRotAxis
      () -> !TwistJoystick.getRawButton(2)
      ));

    liftSubsystem.setDefaultCommand(new LiftJoystickCommand(
      liftSubsystem,
      () -> RobotMechJoystick.getRawAxis(1),
      () -> RobotMechJoystick.getRawAxis(5),
      () -> RobotMechJoystick.getRawButton(6)
      ));



    configureButtonBindings();
    
    m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", m_chooser);
    limelightSubsystem.setDefaultCommand(new ApriltagCommand(limelightSubsystem));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() { 
        //resets yaw/Heading when button 1 is pressed
        new JoystickButton(TwistJoystick, 3).whileTrue(new RunCommand(
                () ->swerveSubsystem.zeroHeading()));

        new JoystickButton(RobotMechJoystick,5).whileTrue(new IntakeCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,1).whileTrue(new SpeakerShooterCmd(robotMechSubsystem));//, () -> RobotMechJoystick.getRawAxis(3)));
        new JoystickButton(RobotMechJoystick,3).whileTrue(new AmpShooterCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,4).whileTrue(new SourceIntakeCmd(robotMechSubsystem));
        //new JoystickButton(RobotMechJoystick,6).whileTrue(new autoPerspectiveTestCmd(swerveSubsystem));//
        new JoystickButton(RobotMechJoystick,2).whileTrue(new RunCommand( () -> swerveSubsystem.getAbsoluteEncoderPosition()));
        
  }

  public void setIdleModeBrake(){
        swerveSubsystem.idleBrake();
  }
  public void setIdleModeCoast(){
        swerveSubsystem.idleCoast();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommandAmp2() {
        return new PathPlannerAuto("2AmpAuto");
      }

   public Command getAutonomousCommandAmp3() {
        return new PathPlannerAuto("3AmpAuto");
      }

   public Command getAutonomousCommandCenter2() {
        return new PathPlannerAuto("2CenterAuto");
      }

   public Command getAutonomousCommandCenter3() {
        return new PathPlannerAuto("3CenterAuto");
      }

   public Command getAutonomousCommandSource2() {
        return new PathPlannerAuto("2SourceAuto");
   }

   public Command getAutonomousCommandDestroy() {
        return new PathPlannerAuto("Destroy");
      }

   public Command getAutonomousCommandSource3() {
        return new PathPlannerAuto("3SourceAuto");
      }

   public Command getAutonomousCommandShootAndStay() {
        return new PathPlannerAuto("ShootAndStay");
      }

   public Command getAutonomousCommandShootAndLeaveSource() {
        return new PathPlannerAuto("ShootAndLeaveSource");
      }

   public Command getAutonomousCommandOnTheGo5000() {
        return new PathPlannerAuto("OnTheGo5000");
   }


   /*public Command getAutonomousCommandCenter() {
        // 1. Create trajectory settings
        //System.out.println("getAutonomous Center");
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                3,//AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


       // 2. Generate trajectory
        Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(-.88, .01)   //way point
                ),
                new Pose2d(-2, .01, Rotation2d.fromDegrees(0)), //final point was -1.76,.01
                trajectoryConfig);   //trajectory config

        Trajectory trajectory7 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-2, .001, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(-.88, 0.01)   //way point
                ),
                new Pose2d(0.5, 0, Rotation2d.fromDegrees(0)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory8 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, .01, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(-.88, 0.01)   //way point
                ),
                new Pose2d(-1, 0.01, Rotation2d.fromDegrees(180)), //final point
                trajectoryConfig);   //trajectory config


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
                //trajectory,
                trajectory6,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
                
        SwerveControllerCommand swerveControllerCommand6 = new SwerveControllerCommand(
                //Robot.trajectoryPW1,
                trajectory7,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
                
        SwerveControllerCommand swerveControllerCommand7 = new SwerveControllerCommand(         //Rotate 180deg
                //Robot.trajectoryPW1,
                trajectory8,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);              


        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
               new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory6.getInitialPose())),
                new AutoShooterCmd(robotMechSubsystem),
                new ParallelCommandGroup( 
                        swerveControllerCommand5, 
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new AutoIntakeCmd(robotMechSubsystem))),
                swerveControllerCommand6,
                new AutoShooterCmd(robotMechSubsystem),
                swerveControllerCommand7,                                       
                new InstantCommand(() -> swerveSubsystem.stopModules()),
                new InstantCommand(() -> swerveSubsystem.zeroHeading())         
                ); 
  }*/
}
          