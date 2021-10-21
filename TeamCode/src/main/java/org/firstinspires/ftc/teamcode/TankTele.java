package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TankDrive", group = "Teleop")

public class TankTele extends OpMode {
    private Robot bot;
    public double setPower = 1; // Motor power multiplier

    @Override
    // Initialize the robot (this is what happens when the play button is pressed)
    public void init() {
        bot = new Robot(hardwareMap, telemetry, false);
        bot.initBot();
        telemetry.addData("Bot", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    // loop code to be continuously cycled through after init
    public void loop() {
        leftChangePower();
        rightChangePower();
        // get double numbers for each control to be inputted for the power functions of the bot
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        // if the controls are not at least 15% pushed down, the bot will not move
        //     this is to account for drift in the hardware
        rightTrigger = Math.abs(rightTrigger) > 0.15 ? rightTrigger : 0.0;
        leftTrigger = Math.abs(leftTrigger) > 0.15 ? leftTrigger : 0.0;
        leftStickY = Math.abs(leftStickY) > 0.15 ? leftStickY : 0.0;
        rightStickY = Math.abs(rightStickY) > 0.15 ? rightStickY : 0.0;

        // if the right trigger is pressed down strafe at that power
        if (rightTrigger > 0.0) {
            bot.strafe(rightTrigger);
        } else if (leftTrigger > 0.0) {
            // if the left trigger is pressed down, enter the negative value into the strafe
            //     function to make it strafe left
            bot.strafe(-leftTrigger);
        } else {
            // Otherwise, set the power to whatever the Y sticks are
            bot.drive(rightStickY, leftStickY);
        }
    }

    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;

    // LEFT: change the movement power by -0.1 if the right bumper on the gamepad is pressed, and visa-versa
    public void leftChangePower() {
        if (gamepad1.left_bumper && !leftBumperPressed && setPower > 0) {
            setPower -= 0.1;
            leftBumperPressed = true;
        } else
            leftBumperPressed = false;

        telemetry.addData("left bumper: speed", setPower);
    }

    // RIGHT: change the movement power by -0.1 if the right bumper on the gamepad is pressed, and visa-versa
    public void rightChangePower() {
        if (gamepad1.right_bumper && !rightBumperPressed && setPower < 1) {
            setPower += 0.1;
            rightBumperPressed = true;
        } else
            rightBumperPressed = false;

        telemetry.addData("right bumper: speed", setPower);
    }

    @Override
    // Sets the power of all motors to 0 when stop button is pressed
    public void stop() {
        bot.drive(0.0, 0.0);
    }
}