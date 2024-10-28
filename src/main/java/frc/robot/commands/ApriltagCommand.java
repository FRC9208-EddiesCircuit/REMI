// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class ApriltagCommand extends Command {

  private final LimelightSubsystem limelightSubsystem;

  public ApriltagCommand(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelightSubsystem.getID()==7 ||limelightSubsystem.getID()==4 ){ //side
        if (/*(limelightSubsystem.getTv() == true) && */((limelightSubsystem.getTx() > -1.13) && (limelightSubsystem.getTx() < 1.13)) && ((limelightSubsystem.getTa() > 0.22 && (limelightSubsystem.getTa() < 10)))){
         limelightSubsystem.setPurpleLightsOn();
        }else {
          limelightSubsystem.setPurpleLightsOff();
        }
    } //side speaker
    else if (limelightSubsystem.getID()==5 ||limelightSubsystem.getID()==6 ){ //amp
        if ((limelightSubsystem.getTv() == true) && ((limelightSubsystem.getTx() > -5.0) && (limelightSubsystem.getTx() < 5.0)) && ((limelightSubsystem.getTa() > 1.19 && (limelightSubsystem.getTa() < 10)))){
         limelightSubsystem.setPurpleLightsOn();
        }else {
          limelightSubsystem.setPurpleLightsOff();
        }
      }
     } //amp
    //else{limelightSubsystem.setPurpleLightsOff();}*/
    /*if ((limelightSubsystem.getTv() == true) && ((limelightSubsystem.getTx() > -5.0) && (limelightSubsystem.getTx() < 5.0)) && ((limelightSubsystem.getTa() > 1.19 && (limelightSubsystem.getTa() < 10)))){
      limelightSubsystem.setPurpleLightsOn();
    }else {
      limelightSubsystem.setPurpleLightsOff();
    }
  }*/

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
