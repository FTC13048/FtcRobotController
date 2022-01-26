package org.firstinspires.ftc.teamcode.HardwareStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.function.Function;

public class GamePadEx {
    public final Gamepad gamepad;
    public static final double MIN_THRESHOLD = 0.15;
    private HashMap<ControllerButtons, Boolean> buttons;

    public GamePadEx(Gamepad pad) {
        gamepad = pad;
        buttons = new HashMap<>();
    }

    /**
     * Returns true while the given button is held down
     *
     * @param button The button to check against
     * @return If the button is held down
     */
    public boolean getControl(ControllerButtons button) {
//        boolean gamepadVal = button.function.apply(gamepad);
//
//        if (!buttons.containsKey(button)) {
//            buttons.put(button, true);
//            return true;
//        }
//
//        if (gamepadVal && !buttons.get(button)) {
//            buttons.put(button, true);
//            return true;
//        }
//
//        if (!gamepadVal && buttons.get(button)) {
//            buttons.put(button, false);
//        }
//
//        return false;

        // Simpler version
        boolean gamepadVal = button.function.apply(gamepad);

        buttons.put(button, gamepadVal);

        return gamepadVal;
    }

    /**
     * Returns true if the given button started being held down
     *
     * @param button The button to check against
     * @return If the button started being held down
     */
    public boolean getControlDown(GamePadEx.ControllerButtons button) {
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
     * Returns true if the button stopped being held down
     *
     * @param button The button to check against
     * @return If the button stopped being held down
     */
    public boolean getControlRelease(GamePadEx.ControllerButtons button) {
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

    public enum ControllerButtons {
        A(g -> g.a), B(g -> g.b), X(g -> g.x), Y(g -> g.y), RBUMP(g -> g.right_bumper),
        LBUMP(g -> g.left_bumper), L3(g -> g.left_stick_button), R3(g -> g.right_stick_button),
        SELECT(g -> g.back), DPADUP(g -> g.dpad_up), DPADDOWN(g -> g.dpad_down),
        DPADLEFT(g -> g.dpad_left), DPADRIGHT(g -> g.dpad_right),

        RTRIGGER((g -> g.right_trigger > GamePadEx.MIN_THRESHOLD)),
        LTRIGGER((g -> g.left_trigger > GamePadEx.MIN_THRESHOLD)),
        LSTICKY((g -> g.left_stick_y > GamePadEx.MIN_THRESHOLD)),
        RSTICKY((g -> g.right_stick_y > GamePadEx.MIN_THRESHOLD));

        public Function<Gamepad, Boolean> function;

        ControllerButtons(Function<Gamepad, Boolean> function) { this.function = function; }
    }
}