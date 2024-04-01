// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkFlex shooterLeft = new CANSparkFlex(15, MotorType.kBrushless);
  private CANSparkFlex shooterRight = new CANSparkFlex(16, MotorType.kBrushless);
  
  public void revShooters() {
    shooterLeft.set(0.70);
    shooterRight.set(-1);
  }

  public void stopShooters() {
    shooterLeft.set(0);
    shooterRight.set(0);
  }

  public void customRev(double speed) {
    shooterLeft.set(speed);
    shooterRight.set(speed * -1);
  }
}
