// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ampModeArm;
import frc.robot.commands.backOutNote;
import frc.robot.commands.moveArmToZero;
import frc.robot.commands.moveWristToAmp;
import frc.robot.commands.moveWristToIntake;
import frc.robot.commands.moveWristToShoot;
import frc.robot.commands.moveWristToZero;
import frc.robot.commands.outtakeNote;
import frc.robot.commands.shoot;
import frc.robot.commands.zeroArm;
import frc.robot.commands.zeroWrist;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.lang.reflect.Constructor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerve;
  XboxContainer controls;
  ArmSubsystem arm;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SwerveSubsystem swerveSub, XboxContainer xboxSub, ArmSubsystem armSub, IntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
    // Configure the trigger bindings
    swerve = swerveSub;
    controls = xboxSub;
    arm = armSub;
    intake = intakeSub;
    shooter = shooterSub;
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controls.orientedToggle.onTrue(swerve.toggleOrientedMode());
    controls.moveToZero.onTrue(intake.disableIntake().andThen(new moveWristToZero(arm)).andThen(new moveArmToZero(arm)));
    controls.zeroEncoders.onTrue(intake.disableIntake().andThen(new zeroWrist(arm)).andThen(new zeroArm(arm)));
    controls.ampMode.onTrue(
      intake.disableIntake()
      .andThen(new ampModeArm(arm))
      .andThen(new moveWristToAmp(arm))
      );
    controls.outtake.onTrue(
      new outtakeNote(intake, shooter)
      .andThen(new moveWristToZero(arm))
      .andThen(new moveArmToZero(arm))
      );
    controls.intakeMode.onTrue(
      new moveWristToZero(arm)
      .andThen(new moveArmToZero(arm))
      .andThen(new moveWristToIntake(arm, intake))
      .andThen(intake.enableIntake())
      .andThen(new WaitUntilCommand(intake::readBeamBreak))
      .andThen(intake.disableIntake())
      .andThen(new moveWristToZero(arm))
      /*.andThen(new backOutNote(intake))
      .andThen(new WaitCommand(1))
      .andThen(new moveWristToShoot(arm, intake))
      .andThen(new shoot(intake, shooter))
      .andThen(new moveWristToZero(arm, intake))*/
    );
    controls.shoot.onTrue(
      new backOutNote(intake, shooter)
      .andThen(new moveWristToShoot(arm, intake, shooter))
      .andThen(new shoot(intake, shooter))
      .andThen(new moveWristToZero(arm))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
