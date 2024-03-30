// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class moveWristToZero extends Command {
  private ArmSubsystem arm;
  private IntakeSubsystem intake;
  /** Creates a new moveWristToZero. */
  public moveWristToZero(ArmSubsystem armSub, IntakeSubsystem intakeSub) {
    arm = armSub;
    intake = intakeSub;
    addRequirements(arm, intake);
  }

  @Override public void initialize() {
  }

  @Override public void execute() {
    arm.setWristPosition(0);
  }

  @Override public boolean isFinished() {
    return Math.abs(arm.getWristPosition()) < 2;
  }
}
