// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(17);
  public DigitalInput intakeBeamBreak = new DigitalInput(1);
  public boolean isIntaking = false;

  public void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public Command toggleIntake() {
    return this.runOnce(() -> isIntaking = !isIntaking);
  }
}
