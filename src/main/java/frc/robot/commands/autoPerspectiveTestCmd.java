// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class autoPerspectiveTestCmd extends Command {
  /** Creates a new autoPerspectiveTestCmd. */
  private final SwerveSubsystem swerveSubsystem;

  public autoPerspectiveTestCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
    double startTimer;
    double elapsedTime;
    boolean ah = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        elapsedTime = Timer.getFPGATimestamp() - startTimer;


        swerveSubsystem.driveSwerve(1, 0, 0, true);

        if(elapsedTime >= 5) ah = true;
        else ah = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ah;
  }
}
