// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkFlex bigPivotNormal = new CANSparkFlex(12, MotorType.kBrushless);
  private CANSparkFlex bigPivotInverted = new CANSparkFlex(13, MotorType.kBrushless);
  private SparkPIDController bigPivotNormalPID = bigPivotNormal.getPIDController();
  private SparkPIDController bigPivotInvertedPID = bigPivotInverted.getPIDController();
  private RelativeEncoder bigPivotNormalEncoder = bigPivotNormal.getEncoder();
  private RelativeEncoder bigPivotInvertedEncoder = bigPivotInverted.getEncoder();
  public double armPos = 0;
  private double zeroArmTimer = 0;

  private double lastEncoderPosition = 0;
  private boolean finishedZeroing = false;

  public void initPID() {
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

  public void setBigPivotPosition(double position) {
    SmartDashboard.putNumber("Desired Pos", position);
    bigPivotNormalPID.setReference(position, CANSparkFlex.ControlType.kPosition);
    bigPivotInvertedPID.setReference(-position, CANSparkFlex.ControlType.kPosition);
    armPos = position;
  }

  public double getBigPivotPosition() {
    return bigPivotNormalEncoder.getPosition();
  }

  public Command setToZero() {
    return this.runOnce(() -> {
      while (Math.abs(bigPivotNormalEncoder.getPosition()) > 0.1) {
        bigPivotNormalEncoder.setPosition(0);
        bigPivotInvertedEncoder.setPosition(0);
      }
      armPos = 0;
    });
  }

  public Command ampMode() {
    return this.runOnce(() -> setBigPivotPosition(-10.8));
  }

  public Command moveToZero() {
    return this.runOnce(() -> setBigPivotPosition(0));
  }

  public Command zeroBigPivot() {
    return new Command() { 
      @Override
      public void initialize() {
        zeroArmTimer = System.currentTimeMillis();
        finishedZeroing = false;
        SmartDashboard.putBoolean("finishedZero", finishedZeroing);
      }
      
      @Override
      public void execute() {
        setBigPivotPosition(armPos + 0.2);
        if (System.currentTimeMillis() - zeroArmTimer > 1000) {
          if (Math.abs(bigPivotNormalEncoder.getPosition() - lastEncoderPosition) < 0.025) {
            // zero the encoder
            while (Math.abs(bigPivotNormalEncoder.getPosition()) > 0.1) {
              bigPivotNormalEncoder.setPosition(0);
              bigPivotInvertedEncoder.setPosition(0);
            }
            SmartDashboard.putBoolean("finishedZero", true);
            finishedZeroing = true;
          }
        }
        lastEncoderPosition = bigPivotNormalEncoder.getPosition();
      }

      @Override
      public void end(boolean interrupted) {
        setBigPivotPosition(0);
      }

      @Override
      public boolean isFinished() {
        return finishedZeroing;
      }
    };
  }
}
