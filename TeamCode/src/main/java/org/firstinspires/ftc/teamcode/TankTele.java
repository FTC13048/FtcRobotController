package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "TankDrive", group = "Teleop")

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
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    // loop code to be continuously cycled through after init
    public void loop() {
//        leftChangePower();
//        rightChangePower();

        // get double numbers for each control to be inputted for the power functions of the bot
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickY = gamepad1.right_stick_y;

        double intake = gamepad2.right_trigger;
        double outtake = gamepad2.left_trigger;
        double duckPower = gamepad2.left_stick_y;
        double slidePower = gamepad2.right_stick_y;

        // if the controls are not at least 15% pushed down, the bot will not move
        //     this is to account for drift in the hardware
        rightTrigger = Math.abs(rightTrigger) > 0.15 ? rightTrigger : 0.0;
        leftTrigger = Math.abs(leftTrigger) > 0.15 ? leftTrigger : 0.0;
        leftStickY = Math.abs(leftStickY) > 0.15 ? leftStickY : 0.0;
        rightStickY = Math.abs(rightStickY) > 0.15 ? rightStickY : 0.0;

        intake = intake > 0.15 ? intake : 0.0;
        duckPower = Math.abs(duckPower) > 0.15 ? duckPower : 0.0;
        slidePower = Math.abs(slidePower) > 0.15 ? slidePower : 0.0;

        updateServo();
        updateDriving(rightTrigger, leftTrigger, leftStickY, rightStickY);
        updateSystem(intake, outtake, duckPower, slidePower);
        telemetry.update();
    }

    public void updateDriving(double rTrigger, double lTrigger, double lStickY, double rStickY){
        // if the right trigger is pressed down strafe at that power
        if (rTrigger > 0.0) {
            bot.strafe(rTrigger);
        } else if (lTrigger > 0.0) {
            // if the left trigger is pressed down, enter the negative value into the strafe
            //     function to make it strafe left
            bot.strafe(-lTrigger);
        } else {
            // Otherwise, set the power to whatever the Y sticks are
            bot.drive(rStickY, lStickY);
        }
    }

    public void updateSystem(double intake, double outtake, double duck, double slide){
        // if either the left or right triggers on gamepad two are pressed, run the
        //    intake at that power
        if(intake > 0.0){
            bot.runIntake(intake);
        } else if(outtake > 0.0) {
                bot.runIntake(-outtake);
        } else{ bot.runIntake(0.0); }

        if(Math.abs(duck) > 0.0){ bot.runDuckSpinner(duck); }
        else{ bot.runDuckSpinner(0.0); }

        if(Math.abs(slide) > 0.0){ bot.runLinSlide(slide * 0.3); }
        else{ bot.runLinSlide(0.0); }
    }

    private boolean debounceArm = false;
    private int armPos = -1;

    private void updateServo(){
        // if a is pressed, the arm can only switch position if the debounce variable if false
        //     if it is, then it will become true after the  position switches, allowing the
        //     player to hold down the button without it trying to switch positions
        if(gamepad2.a){
            telemetry.addData("a", "pressed");
            // Arm goes to pushing position if it is at rest
            if(!debounceArm && armPos == -1){
                bot.cargoFlipper.setPosition(0.9);
                telemetry.addData(">", "0.9");
                armPos *= -1;
            }

            // Arm goes to rest position if a is pressed an it is pushing
            else if(!debounceArm && armPos == 1){
                bot.cargoFlipper.setPosition(0.1);
                telemetry.addData(">", "0.1");
                armPos *= -1;
            }

            debounceArm = true;
        } else{ debounceArm = false; }
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