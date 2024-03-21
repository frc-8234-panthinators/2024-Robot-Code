// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class zeroWrist extends Command {
  private ArmSubsystem arm;
  /** Creates a new zeroWrist. */
  public zeroWrist(ArmSubsystem armSub) {
    arm = armSub;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
      public void initialize() {
        arm.finishedZeroingWrist = false;
        SmartDashboard.putBoolean("finishedZeroWrist", arm.finishedZeroingWrist);
      }
      
      @Override
      public void execute() {
        arm.setWristPosition(arm.wristPos - 0.1);
        if (arm.wristLimitSwitch.get()) {
          // zero the encoder
          while (Math.abs(arm.getWristPosition()) > 0.1) {
            arm.wrist.setPosition(0);
          }
          SmartDashboard.putBoolean("finishedZeroWrist", true);
          arm.finishedZeroingWrist = true;
        }
      }

      @Override
      public void end(boolean interrupted) {
        arm.setWristPosition(0);
      }

      @Override
      public boolean isFinished() {
        return arm.finishedZeroingWrist;
      }
}
