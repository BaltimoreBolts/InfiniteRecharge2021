package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.Controller;
import edu.wpi.first.wpilibj.XboxController;

/** A {@link Button} that gets its state from a {@link XboxController}. */
public class TriggerButton extends Button {

    XboxController controller;
    TriggerSelection selection;
    final double threshold = 0.8; // threshold for how far the trigger needs to be pressed to register

    public TriggerButton(XboxController controller, TriggerSelection selection) {
        this.controller = controller;
        this.selection = selection;
    }

    public static enum TriggerSelection {
        LEFT(Constants.Controller.XBOX.TRIGGER.LEFT), RIGHT(Constants.Controller.XBOX.TRIGGER.RIGHT);

        int selection;

        private TriggerSelection(int selection) {
             this.selection = selection;
        }
    }

    @Override
    public boolean get() {
        double leftTriggerValue = controller.getRawAxis(Constants.Controller.XBOX.TRIGGER.LEFT);
        double rightTriggerValue = controller.getRawAxis(Constants.Controller.XBOX.TRIGGER.RIGHT);
        double delta = rightTriggerValue - leftTriggerValue;

        // if the axis difference is less than the negative threshold and the left trigger is the selection
        // or if the axis difference is greater than the threshold and right trigger is the selection
        return (
            delta < -threshold && selection.selection == Controller.XBOX.TRIGGER.LEFT
            || delta > threshold && selection.selection == Controller.XBOX.TRIGGER.RIGHT
        );
    }
}
