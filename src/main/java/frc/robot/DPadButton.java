package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.XboxController;

/** A {@link Button} that gets its state from a {@link XboxController}. */
public class DPadButton extends Button {

    XboxController controller;
    Direction direction;

    public DPadButton(XboxController controller, Direction direction) {
        this.controller = controller;
        this.direction = direction;
    }

    public static enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
    }

    @Override
    public boolean get() {
        int dPadValue = controller.getPOV();
        return (dPadValue == direction.direction) 
            || (dPadValue == (direction.direction + 45) % 360)
            || (dPadValue == (direction.direction + 315) % 360);
    }
}
