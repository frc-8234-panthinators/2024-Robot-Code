package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
    public CANSparkMax climberMotor = new CANSparkMax(20, MotorType.kBrushless);
    public SparkPIDController climberPID = climberMotor.getPIDController();
    public RelativeEncoder climberEncoder = climberMotor.getEncoder();
    public double climberPos = 0;
    
    public void initPID() {
        climberMotor.restoreFactoryDefaults();
        climberMotor.setInverted(true);
        climberPID.setP(0.1);
        climberPID.setI(0.0);
        climberPID.setD(0.0);
        climberPID.setIZone(0);
        climberPID.setFF(0.0);
        climberPID.setOutputRange(-0.2, 0.2);
    }

    public void setClimberPosition(double position) {
        SmartDashboard.putNumber("Climber Pos", position);
        climberPID.setReference(position, CANSparkFlex.ControlType.kPosition);
        climberPos = position;
    }
    
    public void moveClimber(double speed) {
        climberMotor.set(speed);
    }
    
    public double getClimberPosition() {
        return climberEncoder.getPosition();
    }
}
