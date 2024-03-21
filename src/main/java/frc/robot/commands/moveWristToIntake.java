// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class moveWristToIntake extends Command {
  ArmSubsystem arm;
  IntakeSubsystem intake;
  /** Creates a new moveWristToIntake. */
  public moveWristToIntake(ArmSubsystem armSub, IntakeSubsystem intakeSub) {
    arm = armSub;
    intake = intakeSub;
    addRequirements(arm, intake);
  }

  // Called when the command is initially scheduled.
  @Override public void initialize() {
    if (!intake.isIntaking) intake.toggleIntake().schedule();
  }

  @Override public void execute() {
    arm.setWristPosition(27);
  }

  @Override public boolean isFinished() {
    return Math.abs(arm.getWristPosition()-27) < 2;
  }
}
