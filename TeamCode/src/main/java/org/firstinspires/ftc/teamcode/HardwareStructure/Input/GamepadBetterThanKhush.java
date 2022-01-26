package org.firstinspires.ftc.teamcode.HardwareStructure.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.LinkedHashMap;

public class GamepadBetterThanKhush {
    public Gamepad thisGamepad;

    public static final double MIN_THRESHOLD = 0.15;

    public HashMap<Button, Boolean> ButtonStates = new LinkedHashMap<>();

    /**
     * Creates a new gamepad object with a set name
     *
     * @param gamepad The actual gamepad to initialize it with
     */
    public GamepadBetterThanKhush(Gamepad gamepad) {
        this.thisGamepad = gamepad;
    }

    /**
     * Returns true if the given button is held down
     *
     * @param button The button to check against
     * @return If the button is held down
     */
    public boolean getButton(Button button) {
        boolean linkedValue = button.buttonFunction.apply(thisGamepad);

        ButtonStates.put(button, linkedValue);

        return linkedValue;
    }

    /**
     * Returns true if the given button started being held down
     *
     * @param button The button to check against
     * @return If the button started being held down
     */
    public boolean getButtonDown(Button button) {
        boolean linkedValue = button.buttonFunction.apply(thisGamepad);

        if (!ButtonStates.containsKey(button)) {
            ButtonStates.put(button, linkedValue);
            return linkedValue;
        }

        if (!ButtonStates.get(button) && linkedValue) {
            ButtonStates.put(button, linkedValue);
            return true;
        } else {
            ButtonStates.put(button, linkedValue);
            return false;
        }
    }

    /**
     * Returns true if the button stopped being held down
     *
     * @param button The button to check against
     * @return If the button stopped being held down
     */
    public boolean getButtonUp(Button button) {
        boolean linkedValue = button.buttonFunction.apply(thisGamepad);

        if (!ButtonStates.containsKey(button)) {
            ButtonStates.put(button, linkedValue);
            return false;
        }

        if (ButtonStates.get(button) && !linkedValue) {
            ButtonStates.put(button, linkedValue);
            return true;
        } else {
            ButtonStates.put(button, linkedValue);
            return false;
        }
    }
}