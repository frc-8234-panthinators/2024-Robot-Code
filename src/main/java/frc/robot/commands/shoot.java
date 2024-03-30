// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.ml.StatModel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shoot extends Command {
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  double startTime = 0;
  public shoot(IntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
    intake = intakeSub;
    shooter = shooterSub;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.revShooters();
    if ((System.currentTimeMillis() - startTime) > 1000) {
      intake.setIntake(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooters();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > 1500;
  }
}
