package org.firstinspires.ftc.teamcode.HardwareStructure.Input;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

public enum Button {
    // A, B, X, Y, RBUMP, LBUMP, L3, R3, SELECT, DPADUP, DPADDOWN, DPADLEFT, DPADRIGHT

    A(g -> g.a),
    B(g -> g.b),
    X(g -> g.x),
    Y(g -> g.y),

    RBUMP(g -> g.right_bumper),
    LBUMP(g -> g.left_bumper),

    L3(g -> g.left_stick_button),
    R3(g -> g.right_stick_button),
    SELECT(g -> g.back),

    DPADUP(g -> g.dpad_up),
    DPADDOWN(g -> g.dpad_down),
    DPADLEFT(g -> g.dpad_left),
    DPADRIGHT(g -> g.dpad_right),

    RTRIGGER((g -> g.right_trigger > GamepadBetterThanKhush.MIN_THRESHOLD)),
    LTRIGGER((g -> g.left_trigger > GamepadBetterThanKhush.MIN_THRESHOLD)),

    LSTICKY((g -> g.left_stick_y > GamepadBetterThanKhush.MIN_THRESHOLD)),
    RSTICKY((g -> g.right_stick_y > GamepadBetterThanKhush.MIN_THRESHOLD));

    Function<Gamepad, Boolean> buttonFunction;

    Button(Function<Gamepad, Boolean> function) {
        this.buttonFunction = function;
    }
}