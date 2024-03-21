// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class zeroArm extends Command {
  private ArmSubsystem arm;
  /** Creates a new zeroArm. */
  public zeroArm(ArmSubsystem armSub) {
    arm = armSub;
    addRequirements(arm);
  }

  
  @Override
  public void initialize() {
    arm.zeroArmTimer = System.currentTimeMillis();
    arm.finishedZeroingBigPivot = false;
    SmartDashboard.putBoolean("finishedZeroArm", arm.finishedZeroingBigPivot);
    arm.bigPivotNormalPID.setOutputRange(-0.2, 0.2);
    arm.bigPivotInvertedPID.setOutputRange(-0.2, 0.2);
  }
  
  @Override
  public void execute() {
    arm.setBigPivotPosition(arm.armPos + 0.2);
    if (System.currentTimeMillis() - arm.zeroArmTimer > 1000) {
      if (Math.abs(arm.bigPivotNormalEncoder.getPosition() - arm.lastArmEncoderPosition) < 0.025) {
        // zero the encoder
        while (Math.abs(arm.bigPivotNormalEncoder.getPosition()) > 0.1) {
          arm.bigPivotNormalEncoder.setPosition(0);
          arm.bigPivotInvertedEncoder.setPosition(0);
        }
        SmartDashboard.putBoolean("finishedZeroArm", true);
        arm.finishedZeroingBigPivot = true;
      }
    }
    arm.lastArmEncoderPosition = arm.bigPivotNormalEncoder.getPosition();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setBigPivotPosition(0);
  }

  @Override
  public boolean isFinished() {
    return arm.finishedZeroingBigPivot;
  }
}
