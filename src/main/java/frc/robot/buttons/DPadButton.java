package frc.robot.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.DriverStation;

public class DPadButton extends Button {

    Joystick joystick;
    Direction direction;
    private int port;

    public DPadButton(int port, Direction direction) {
        this.direction = direction;
        this.port = port;
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
        int dPadValue = DriverStation.getStickPOV(port, 0); //Second Value is default index
        return (dPadValue == direction.direction);
    }

}
