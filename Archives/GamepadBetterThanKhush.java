package org.firstinspires.ftc.teamcode.Archives;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.GamePadEx;

import java.util.HashMap;

public class GamepadBetterThanKhush {
    public Gamepad thisGamepad;

    public static final double minimumThreshold = 0.15;

    public HashMap<GamePadEx.ControllerButton, Boolean> ButtonStates = new HashMap<>();

    /**
     * Creates a new GamepadBetterThanKhush object with a set name
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
    public boolean getButton(GamePadEx.ControllerButton button) {
        boolean linkedValue = button.function.apply(thisGamepad);

        ButtonStates.put(button, linkedValue);

        return linkedValue;
    }

    /**
     * Returns true if the given button started being held down
     *
     * @param button The button to check against
     * @return If the button started being held down
     */
    public boolean getButtonDown(GamePadEx.ControllerButton button) {
        boolean linkedValue = button.function.apply(thisGamepad);

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
    public boolean getButtonUp(GamePadEx.ControllerButton button) {
        boolean linkedValue = button.function.apply(thisGamepad);

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