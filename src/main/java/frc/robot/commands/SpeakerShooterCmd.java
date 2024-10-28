// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;

public class SpeakerShooterCmd extends Command {
  /** Creates a new SpeakerShooterCmd. */
  private final RobotMechSubsystem robotMechSubsystem;
  //private final Supplier<Double> Axis;

  public double startTimer;
  public double elapsedTime;

  public double axis;
  public boolean triggerBoolean;
  public boolean axisBoolean;


  public SpeakerShooterCmd(RobotMechSubsystem robotMechSubsystem){//, Supplier<Double> Axis) {
    this.robotMechSubsystem = robotMechSubsystem;
    //this.Axis = Axis;
    addRequirements(robotMechSubsystem);
  }

  public boolean getBoolean(double axis){
    if (axis > .1) triggerBoolean = true;
    else if(axis <= .1) triggerBoolean = false;
    return triggerBoolean;
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

    //axis = Axis.get();
    //axisBoolean = getBoolean(axis);

    robotMechSubsystem.setShooterVoltage(10.5, 10.5);

    if(robotMechSubsystem.getTopShooterVelocity() >= 3300){// && axisBoolean){
      robotMechSubsystem.setIntake(.5, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotMechSubsystem.setIntake(0, 0);
    robotMechSubsystem.setShooterVoltage(0,0);
    robotMechSubsystem.getIntakeSensor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}