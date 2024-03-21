// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOError;
import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.XboxContainer;
import frc.robot.commands.zeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private XboxContainer controls = new XboxContainer();
  private SwerveSubsystem swerve = new SwerveSubsystem();
  private ArmSubsystem arm = new ArmSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private double turnSpeed = 0.25;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(swerve, controls, arm, intake, shooter);
    swerve.init();
    arm.initPID();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    new zeroArm(arm).schedule();
    /*m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    new zeroArm(arm).schedule();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (controls.resetHeading()) {
      swerve.resetHeading();
    }
    if (controls.driveX() > 0.1 || controls.driveY() > 0.1) {
      turnSpeed = 1;
    } else {
      turnSpeed = 0.25;
    }
    if (controls.getManualBigPivotPosition() > 0.1 || controls.getManualBigPivotPosition() < -0.1) {
      arm.wristPos += controls.getManualBigPivotPosition() * 0.5;
    }
    arm.getLimitSwitchStates();
    SmartDashboard.putNumber("Big Pivot Position", arm.getBigPivotPosition());
    SmartDashboard.putNumber("Arm Pos", arm.armPos);
    SmartDashboard.putNumber("Real Wrist Pos", arm.getWristPosition());
    SmartDashboard.putNumber("Desired Wrist Pos", arm.wristPos);
    arm.setWristPosition(arm.wristPos);
    swerve.Drive(controls.calcControllerCurve(controls.driveX(), controls.driveY()), controls.rotation() * turnSpeed);
    swerve.updateOdometry();

    if (controls.revShooters()) {
      shooter.revShooters();
    } else {
      shooter.stopShooters();
    }

    if (intake.isIntaking) {
      intake.setIntake(-1);
    } else {
      intake.setIntake(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}