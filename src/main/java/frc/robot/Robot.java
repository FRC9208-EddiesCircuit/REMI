// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveModule;
//import edu.wpi.first.cameraserver.CameraServer;
/* 
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
//robot
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  private static final String kNothingAuto = "Do Nothing";
  private static final String k2AmpSide = "2 Note Amp Side Auto";
  private static final String k3AmpSide = "3 Note Amp Side Auto";
  private static final String k2Center = "2 Note Center Auto";
  private static final String k3Center = "3 Note Center Auto";
  private static final String k2Source = "2 Note Source Side Auto";
  private static final String kDestroy = "0 Note Destroy Auto";
  private static final String k3Source = "3 Note Source Side Auto";
  private static final String kSourceLeave = "Shoot And Leave Source";
  private static final String kStay = "Shoot And Stay";
  private static final String kOnTheGo = "On The Go 5000 Auto";

  //private static final String kSource = "Source Side Auto";f



  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private RobotContainer m_robotContainer;
  //private SwerveModule m_swerveModule;
 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
   
    m_robotContainer = new RobotContainer();
    //m_swerveModule.setIdleModeBrake();
    //m_robotContainer.AutonInit();
  
    
    m_chooser.setDefaultOption("Default Auto", kNothingAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("2 Note Amp Side Auto ", k2AmpSide);//("Blue Amp / Red Source Side Auto", kAmpSide);
    m_chooser.addOption("3 Note Amp Side Auto", k3AmpSide);
    m_chooser.addOption("2 Note Center Side Auto ", k2Center);
    m_chooser.addOption("3 Note Center Side Auto ", k3Center);
    m_chooser.addOption("2 Source Side Auto", k2Source);//("Blue Source / Red Amp Side Auto", kSource);
    m_chooser.addOption("0 Note Destroy Auto", kDestroy);
    m_chooser.addOption("3 Source Side Auto", k3Source);
    m_chooser.addOption("ShootAndStay", kStay);
    m_chooser.addOption("ShootAndLeaveSource", kSourceLeave);
    m_chooser.addOption("On The Go 5000", kOnTheGo);



    SmartDashboard.putData("Auto choices",m_chooser);
     
    //CameraServer.startAutomaticCapture(0);
    


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_robotContainer.AutonSelector();
    m_robotContainer.setIdleModeBrake();

    
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected:" + m_autoSelected);
    switch (m_autoSelected) {

      case kNothingAuto:
        break;

      case k2AmpSide:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandAmp2();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      case k3AmpSide:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandAmp3();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      case k2Center:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandCenter2();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      case k3Center:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandCenter3();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case k2Source:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandSource2();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case kSourceLeave:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandShootAndLeaveSource();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      case kDestroy:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandDestroy();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      case k3Source:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandSource3();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case kStay:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandShootAndStay();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case kOnTheGo:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandOnTheGo5000();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      default:
        break;
    }
   
    /*
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("auto init");
      m_autonomousCommand.schedule();
    }
    */

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setIdleModeCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
