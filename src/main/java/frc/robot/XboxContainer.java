package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class XboxContainer {
    private XboxController controller = new XboxController(0);

    public double driveY() {
        if (Math.abs(controller.getLeftY()) > 0.2) {
            return controller.getLeftY();
        } else {
            return 0d;
        }
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) > 0.2) {
            return controller.getLeftX();
        } else {
            return 0d;
        }
    }

    public double rotation() {
        if (Math.abs(controller.getRightTriggerAxis()) > 0.5) {
            return controller.getRightTriggerAxis();
        } else {
            return 0d;
        }
    }
}
