// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class moveArmToZero extends Command {
  ArmSubsystem arm;
  /** Creates a new moveArmToZero. */
  public moveArmToZero(ArmSubsystem armSub) {
    arm = armSub;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override public void initialize() {
    //set pid speed lower
    arm.bigPivotNormalPID.setOutputRange(-0.1, 0.1);
    arm.bigPivotInvertedPID.setOutputRange(-0.1, 0.1);
  }

  @Override public void execute() {
    arm.setBigPivotPosition(0);
  }

  @Override public boolean isFinished() {
    return Math.abs(arm.getBigPivotPosition()) < 0.5;
  }
}
