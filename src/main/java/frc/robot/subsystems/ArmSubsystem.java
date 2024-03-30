// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  public DigitalInput wristLimitSwitch = new DigitalInput(0);
  public CANSparkFlex bigPivotNormal = new CANSparkFlex(12, MotorType.kBrushless);
  public CANSparkFlex bigPivotInverted = new CANSparkFlex(13, MotorType.kBrushless);
  public TalonFX wrist = new TalonFX(14);
  public SparkPIDController bigPivotNormalPID = bigPivotNormal.getPIDController();
  public SparkPIDController bigPivotInvertedPID = bigPivotInverted.getPIDController();
  public RelativeEncoder bigPivotNormalEncoder = bigPivotNormal.getEncoder();
  public RelativeEncoder bigPivotInvertedEncoder = bigPivotInverted.getEncoder();
  public Slot0Configs slot0Configs = new Slot0Configs();
  public double armPos = 0;
  public double wristPos = 0;
  public double zeroArmTimer = 0;
  public double lastArmEncoderPosition = 0;
  public boolean finishedZeroingWrist = false;
  public boolean finishedZeroingBigPivot = false;
  
  public void initPID() {
    slot0Configs.kP = 0.4;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    wrist.setNeutralMode(NeutralModeValue.Brake);

    wrist.getConfigurator().apply(slot0Configs);

    bigPivotNormal.restoreFactoryDefaults();
    bigPivotInverted.restoreFactoryDefaults();

    double kP = 0.1;
    double kI = 0.0;
    double kD = 0.0;
    double kIz = 0.0;
    double kFF = 0.0;
    double kMaxOutput = 0.2;
    double kMinOutput = -0.2;

    bigPivotNormalPID.setP(kP);
    bigPivotNormalPID.setI(kI);
    bigPivotNormalPID.setD(kD);
    bigPivotNormalPID.setIZone(kIz);
    bigPivotNormalPID.setFF(kFF);
    bigPivotNormalPID.setOutputRange(kMinOutput, kMaxOutput);

    bigPivotInvertedPID.setP(kP);
    bigPivotInvertedPID.setI(kI);
    bigPivotInvertedPID.setD(kD);
    bigPivotInvertedPID.setIZone(kIz);
    bigPivotInvertedPID.setFF(kFF);
    bigPivotInvertedPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void getLimitSwitchStates() {
    SmartDashboard.putBoolean("Wrist Limit Switch", wristLimitSwitch.get());
  }

  public void setBigPivotPosition(double position) {
    SmartDashboard.putNumber("Desired Pos", position);
    bigPivotNormalPID.setReference(position, CANSparkFlex.ControlType.kPosition);
    bigPivotInvertedPID.setReference(-position, CANSparkFlex.ControlType.kPosition);
    armPos = position;
  }

  public void setWristPosition(double position) {
    final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);
    wrist.setControl(posRequest.withPosition(position));
    wristPos = position;
  }

  public double getBigPivotPosition() {
    return bigPivotNormalEncoder.getPosition();
  }

  public double getWristPosition() {
    return wrist.getPosition().refresh().getValue();
  }

  public Command setToZero() {
    return this.runOnce(() -> {
      while (Math.abs(getWristPosition()) > 0.1) {
        wrist.setPosition(0);
      }
      wristPos = 0;
    });
  }
}
