package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.function.Function;

public class GamePadEx {
    private final Gamepad gamepad; // JUST USE THE WRAPPER METHODS DONT MAKE THIS PUBLIC AAAAAAAAAH
    public static final double MIN_THRESHOLD = 0.15;
    private HashMap<ControllerButton, Boolean> buttons;

    public GamePadEx(Gamepad pad) {
        gamepad = pad;
        buttons = new HashMap<>();
    }

    /**
     * Returns true while the given controller button is held down
     *
     * @param button The button to check against
     * @return If the button is held down
     */
    public boolean getControl(ControllerButton button) {
        // Simpler version
        boolean gamepadVal = button.function.apply(gamepad);

        buttons.put(button, gamepadVal);

        return gamepadVal;
    }

    /**
     * Returns true if the given controller button started being held down
     *
     * @param button The button to check against
     * @return If the button started being held down
     */
    public boolean getControlDown(ControllerButton button) {
        boolean linkedValue = button.function.apply(gamepad);

        if (!buttons.containsKey(button)) {
            buttons.put(button, linkedValue);
            return linkedValue;
        }

        if (!buttons.get(button) && linkedValue) {
            buttons.put(button, linkedValue);
            return true;
        } else {
            buttons.put(button, linkedValue);
            return false;
        }
    }

    /**
     * Returns true if the controller button stopped being held down
     *
     * @param button The button to check against
     * @return If the button stopped being held down
     */
    public boolean getControlRelease(ControllerButton button) {
        boolean linkedValue = button.function.apply(gamepad);

        if (!buttons.containsKey(button)) {
            buttons.put(button, linkedValue);
            return false;
        }

        if (buttons.get(button) && !linkedValue) {
            buttons.put(button, linkedValue);
            return true;
        } else {
            buttons.put(button, linkedValue);
            return false;
        }
    }

    /**
     * Returns the value of a desired controller axis
     *
     * @param axis The axis to get its value of
     * @return The value of the axis
     */
    public double getAxis(ControllerAxis axis){
        return (double)axis.function.apply(gamepad);
    }

    public enum ControllerAxis {
        LEFT_X(g -> g.left_stick_x),
        LEFT_Y(g -> g.left_stick_y),
        RIGHT_X(g -> g.right_stick_x),
        RIGHT_Y(g -> g.right_stick_y),
        RIGHT_TRIGGER(g -> g.right_trigger),
        LEFT_TRIGGER(g -> g.left_trigger);

        public Function<Gamepad, Float> function;

        ControllerAxis(Function<Gamepad, Float> function) { this.function = function; }
    }

    public enum ControllerButton {
        A(g -> g.a), B(g -> g.b), X(g -> g.x), Y(g -> g.y), RBUMP(g -> g.right_bumper),
        LBUMP(g -> g.left_bumper), L3(g -> g.left_stick_button), R3(g -> g.right_stick_button),
        BACK(g -> g.back), DPADUP(g -> g.dpad_up), DPADDOWN(g -> g.dpad_down),
        DPADLEFT(g -> g.dpad_left), DPADRIGHT(g -> g.dpad_right), GUIDE(g -> g.guide),

        RTRIGGER((g -> g.right_trigger > GamePadEx.MIN_THRESHOLD)),
        LTRIGGER((g -> g.left_trigger > GamePadEx.MIN_THRESHOLD)),
        LSTICKY((g -> Math.abs(g.left_stick_y) > GamePadEx.MIN_THRESHOLD)),
        RSTICKY((g -> Math.abs(g.right_stick_y) > GamePadEx.MIN_THRESHOLD));

        public Function<Gamepad, Boolean> function;

        ControllerButton(Function<Gamepad, Boolean> function) { this.function = function; }
    }
}