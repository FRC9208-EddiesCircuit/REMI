// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.RobotMechSubsystem;

public class AutoShooterCmd extends Command {

  /** Creates a new AutoShooterCommad.**/

  private final RobotMechSubsystem robotMechSubsystem;
  double startTimer;
  double elapsedTime;
   public AutoShooterCmd(RobotMechSubsystem robotMechSubsystem) {
      this.robotMechSubsystem = robotMechSubsystem;
    addRequirements(robotMechSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsedTime = Timer.getFPGATimestamp() - startTimer;
    robotMechSubsystem.setShooter(.92, .92);

    if(robotMechSubsystem.getBottomShooterVelocity() >= 4000 && elapsedTime < 2){
          robotMechSubsystem.setIntake(1,0);
    }else if(elapsedTime >= 3){
      robotMechSubsystem.setShooter(0,0);
      robotMechSubsystem.setIntake(0,0);
      
    }
  }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      robotMechSubsystem.setShooter(0,0);
      robotMechSubsystem.setIntake(0,0);
      robotMechSubsystem.getIntakeSensor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elapsedTime > 1.25){
      return true;
    }else{
      return false;
    }
  }
}
