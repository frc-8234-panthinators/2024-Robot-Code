package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxContainer {
    private XboxController controller = new XboxController(0);
    private XboxController otherController = new XboxController(1);
    public Trigger orientedToggle = new JoystickButton(controller, XboxController.Button.kY.value);
    public Trigger moveToZero = new JoystickButton(otherController, XboxController.Button.kX.value);
    public Trigger zeroEncoders = new JoystickButton(otherController, XboxController.Button.kB.value);
    public Trigger ampMode = new JoystickButton(otherController, XboxController.Button.kA.value);
    public Trigger intakeMode = new JoystickButton(otherController, XboxController.Button.kY.value);
    public double driveY() {
        if (Math.abs(controller.getLeftY()) > 0.25) {
            return controller.getLeftY();
        } else {
            return 0d;
        }
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) > 0.25) {
            return controller.getLeftX();
        } else {
            return 0d;
        }
    }

    public double rotation() {
        if (Math.abs(controller.getRightX()) > 0.1) {
            return controller.getRightX();
        } else {
            return 0d;
        }
    }

    public Translation2d calcControllerCurve(double x, double y) {
        //double newX = x*Math.pow((Math.pow(x,2)+Math.pow(y,2)),9);
        //double newY = y*Math.pow((Math.pow(x,2)+Math.pow(y,2)),9);
        double newX = x * 0.3;
        double newY = y * 0.3;
        return new Translation2d(newX, newY * -1);
    }

    public boolean revShooters() {
        return otherController.getRightBumper();
    }
    
    public double revIntake() {
        if (otherController.getPOV() == 0) {
            return -1;
        } else if (otherController.getPOV() == 180) {
            return 0.1;
        } else {
            return 0;
        }
    }

    public boolean resetHeading() {
        return controller.getXButton();
    }

    public double getManualBigPivotPosition() {
        return otherController.getRightTriggerAxis() - otherController.getLeftTriggerAxis();
    }
}
