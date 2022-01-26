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

    public boolean controlPressed(ControllerButtons btn) {
        boolean gamepadVal = btn.function.apply(gamepad);

        if (!buttons.containsKey(btn)) {
            buttons.put(btn, true);
            return true;
        }

        if (gamepadVal && !buttons.get(btn)) {
            buttons.put(btn, true);
            return true;
        }

        if (!gamepadVal && buttons.get(btn)) {
            buttons.put(btn, false);
        }

        return false;
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

        Function<Gamepad, Boolean> function;

        ControllerButtons(Function<Gamepad, Boolean> function) {
            this.function = function;
        }
    }
}
