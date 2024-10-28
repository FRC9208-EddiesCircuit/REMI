// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AxisToBooleanSubsystem extends SubsystemBase {
  /** Creates a new AxisToBooleanSubsystem. */
  public AxisToBooleanSubsystem() {}
    boolean triggerBoolean;


  public boolean getBoolean(double axis){
    if (axis > .1) triggerBoolean = true;
    else if(axis <= .1) triggerBoolean = false;
    return triggerBoolean;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
