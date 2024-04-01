// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class outtakeNote extends Command {
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  double startTime = 0;
  /** Creates a new outtakeNote. */
  public outtakeNote(IntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
    intake = intakeSub;
    shooter = shooterSub;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake(0.25);
    shooter.customRev(-0.3);
    if ((System.currentTimeMillis() - startTime) > 1000) {
      intake.setIntake(0);
      shooter.stopShooters();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) > 1200;
  }
}
