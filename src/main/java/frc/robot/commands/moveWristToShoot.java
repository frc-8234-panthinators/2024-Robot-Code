// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class moveWristToShoot extends Command {
  ArmSubsystem arm;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  /** Creates a new moveWristToIntake. */
  public moveWristToShoot(ArmSubsystem armSub, IntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
    arm = armSub;
    intake = intakeSub;
    shooter = shooterSub;
    addRequirements(arm, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override public void initialize() {
    
  }

  @Override public void execute() {
    arm.setWristPosition(11.75);
    shooter.revShooters();
  }

  @Override public boolean isFinished() {
    return Math.abs(arm.getWristPosition()-11.75) < 0.1;
  }
}
