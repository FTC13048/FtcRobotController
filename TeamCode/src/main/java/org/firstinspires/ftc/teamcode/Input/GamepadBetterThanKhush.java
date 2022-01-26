package org.firstinspires.ftc.teamcode.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.ref.Reference;
import java.sql.Ref;
import java.util.HashMap;
import java.util.LinkedHashMap;

public class GamepadBetterThanKhush {

    public String GamepadName;
    public Gamepad thisGamepad;

    public HashMap<Button, Boolean> ButtonStates = new LinkedHashMap<>();

    /**
     * Creates a new gamepad object with a set name
     * @param name The name of this gamepad to initialize it with
     * @return The new gamepad
     */
    public GamepadBetterThanKhush(String name) {
        this.GamepadName = name;
        init();
    }

    public void init(){
        ButtonStates.put(Button.dpad_up, false);
    }

    /**
     * Returns true if the given button is held down
     * @param button The button to check against
     * @return If the button is held down
     */
    public boolean getButton(Button button) {
        thisGamepad.
        return false;
    }

    /**
     * Returns true if the given button started being held down
     * @param button The button to check against
     * @return If the button started being held down
     */
    public boolean getButtonDown(Button button) {

        return false;
    }

    /**
     * Returns true if the button stopped being held down
     *
     * @param button The button to check against
     * @return If the button stopped being held down
     */
    public boolean getButtonUp(Button button) {

        return false;
    }
}