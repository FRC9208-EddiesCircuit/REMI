// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class SetZeroHeadingCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  boolean headingReset;

  /** Creates a new SetZeroHeadingCmd. */
  public SetZeroHeadingCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingReset = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      swerveSubsystem.zeroHeading();
      headingReset = true;
  }


  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(headingReset == true){
      return true;
    }else {
      return false;
    }
  }
}
