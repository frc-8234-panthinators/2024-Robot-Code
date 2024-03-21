// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ampModeArm extends Command {
  private ArmSubsystem arm;
  /** Creates a new ampModeArm. */
  public ampModeArm(ArmSubsystem armSub) {
    arm = armSub;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.bigPivotNormalPID.setOutputRange(-0.2, 0.2);
    arm.bigPivotInvertedPID.setOutputRange(-0.2, 0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setBigPivotPosition(-10.8);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getBigPivotPosition()+10.8) < 0.5;
  }
}
